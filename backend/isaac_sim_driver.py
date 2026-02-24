#!/usr/bin/env python3
"""Isaac Sim direct driver (fallback).

Use this if the USD's ROS2 action graph doesn't work with /joint_states.
This script uses Isaac Sim's Python API to set joint positions directly.

Run inside Isaac Sim's Python environment:
    /opt/isaac-sim/python.sh isaac_sim_driver.py
"""

import logging
import sys
import threading
import time

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger(__name__)

# Add backend to path for imports
sys.path.insert(0, '/opt/digital-twin/backend')

from joint_converter import JOINT_NAMES
from subscriber import IoTSubscriber, TelemetryBuffer


def main():
    # These imports only work inside Isaac Sim's Python environment
    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        import numpy as np
    except ImportError:
        logger.error(
            "This script must be run inside Isaac Sim's Python environment.\n"
            "Usage: /opt/isaac-sim/python.sh isaac_sim_driver.py"
        )
        sys.exit(1)

    # --- Start MQTT subscriber ---
    buffer = TelemetryBuffer()
    subscriber = IoTSubscriber(buffer)

    mqtt_thread = threading.Thread(
        target=lambda: subscriber.start(),
        daemon=True,
        name='mqtt-subscriber',
    )
    mqtt_thread.start()
    time.sleep(2.0)

    # --- Initialize Isaac Sim ---
    USD_PATH = '/opt/digital-twin/assets/SO-ARM101-USD.usd'
    # NOTE: Verify the actual prim path after inspecting the USD file.
    # Common patterns: /World/SO_ARM101, /Root/SO_ARM101, etc.
    ROBOT_PRIM_PATH = '/World/SO_ARM101'

    logger.info("Initializing Isaac Sim world...")
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Open the USD stage
    from omni.isaac.core.utils.stage import open_stage
    open_stage(USD_PATH)

    robot = world.scene.add(
        Articulation(prim_path=ROBOT_PRIM_PATH, name='so_arm101')
    )
    world.reset()

    # Discover joint names in the USD (for mapping)
    usd_joint_names = robot.dof_names
    logger.info(f"USD joint names: {usd_joint_names}")
    logger.info(f"IoT joint names: {JOINT_NAMES}")

    # Build mapping: IoT joint name → USD joint index
    # If names match directly, this is trivial.
    # If not, you'll need to create a manual mapping dict.
    joint_index_map = {}
    for i, usd_name in enumerate(usd_joint_names):
        for iot_name in JOINT_NAMES:
            if iot_name in usd_name.lower() or usd_name.lower() in iot_name:
                joint_index_map[iot_name] = i
                break

    logger.info(f"Joint mapping: {joint_index_map}")

    if len(joint_index_map) < len(JOINT_NAMES):
        logger.warning(
            f"Only mapped {len(joint_index_map)}/{len(JOINT_NAMES)} joints. "
            "Check USD joint names and update mapping manually."
        )

    # --- Main simulation loop ---
    logger.info("Starting simulation loop...")
    last_timestamp = 0

    try:
        while True:
            world.step(render=True)

            latest = buffer.get_latest()
            if latest is None:
                continue

            ts = latest.get('timestamp', 0)
            if ts == last_timestamp:
                continue
            last_timestamp = ts

            positions = latest.get('positions', {})
            num_dofs = robot.num_dof

            # Build position array
            target_positions = np.zeros(num_dofs)
            for iot_name, idx in joint_index_map.items():
                if iot_name in positions:
                    target_positions[idx] = positions[iot_name]

            robot.set_joint_positions(target_positions)

    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        subscriber.stop()
        logger.info("Isaac Sim driver stopped.")


if __name__ == '__main__':
    main()
