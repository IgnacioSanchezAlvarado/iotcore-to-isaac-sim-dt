"""ROS2 JointState publisher for Isaac Sim digital twin.

Reads converted telemetry from TelemetryBuffer and publishes
sensor_msgs/JointState messages to /joint_states topic.
"""

import logging

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from joint_converter import JOINT_NAMES
from subscriber import TelemetryBuffer

logger = logging.getLogger(__name__)


class JointStatePublisher(Node):
    """Publishes joint states from telemetry buffer to ROS2."""

    def __init__(self, buffer: TelemetryBuffer, publish_rate_hz: float = 20.0):
        super().__init__('dt_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.buffer = buffer
        self.last_timestamp = 0
        self._publish_count = 0

        # Poll at 2x telemetry rate to avoid missing updates
        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self._publish_callback)
        self.get_logger().info(
            f'JointState publisher started (poll rate: {publish_rate_hz} Hz)'
        )

    def _publish_callback(self):
        latest = self.buffer.get_latest()
        if latest is None:
            return

        # Skip if no new data
        ts = latest.get('timestamp', 0)
        if ts == self.last_timestamp:
            return
        self.last_timestamp = ts

        positions = latest.get('positions', {})

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = list(JOINT_NAMES)
        msg.position = [positions.get(name, 0.0) for name in JOINT_NAMES]
        # Only send positions — velocity and effort commands conflict with
        # position control and cause physics instability in Isaac Sim.

        self.publisher.publish(msg)
        self._publish_count += 1

        if self._publish_count % 100 == 0:
            self.get_logger().info(f'Published {self._publish_count} joint states')
