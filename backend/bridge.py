#!/usr/bin/env python3
"""Digital Twin Bridge: IoT Core MQTT → ROS2 /joint_states → Isaac Sim.

Single process that:
1. Subscribes to IoT Core MQTT (background thread)
2. Publishes ROS2 JointState messages (main thread via rclpy)

Isaac Sim's USD action graph subscribes to /joint_states and drives the robot.
"""

import logging
import signal
import sys
import threading
import time

import rclpy

from config_loader import load_config
from ros2_publisher import JointStatePublisher
from subscriber import IoTSubscriber, TelemetryBuffer

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger(__name__)


def main():
    config = load_config()
    polling_rate = config.get('iot', {}).get('pollingRateHz', 10)

    # Shared buffer between MQTT subscriber and ROS2 publisher
    buffer = TelemetryBuffer()
    shutdown_event = threading.Event()

    # --- Start MQTT subscriber in background thread ---
    subscriber = IoTSubscriber(buffer)

    def mqtt_thread_fn():
        try:
            subscriber.start()
            logger.info("MQTT subscriber running in background thread")
            shutdown_event.wait()  # Block until shutdown signal
        except Exception as e:
            logger.error(f"MQTT thread error: {e}")
        finally:
            subscriber.stop()

    mqtt_thread = threading.Thread(target=mqtt_thread_fn, name='mqtt-subscriber')
    mqtt_thread.start()

    # Wait for MQTT connection (up to 10s)
    for _ in range(20):
        if buffer.message_count > 0:
            logger.info("First MQTT message received")
            break
        time.sleep(0.5)
    else:
        logger.warning("No MQTT messages received yet — starting ROS2 publisher anyway")

    # --- Start ROS2 publisher in main thread ---
    rclpy.init()
    node = JointStatePublisher(buffer, publish_rate_hz=polling_rate * 2)

    # Graceful shutdown
    def shutdown_handler(signum, frame):
        logger.info("Shutdown signal received")
        shutdown_event.set()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        logger.info("Bridge running. Press Ctrl+C to stop.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, shutting down...")
    finally:
        shutdown_event.set()
        node.destroy_node()
        rclpy.try_shutdown()
        mqtt_thread.join(timeout=5)
        logger.info("Bridge stopped.")


if __name__ == '__main__':
    main()
