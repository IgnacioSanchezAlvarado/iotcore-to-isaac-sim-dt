"""Joint data conversion utilities for LeRobot SO-101 telemetry.

Converts servo tick positions (0-4096) to radians and extracts joint data
from IoT Core telemetry payloads.
"""

import math
from typing import Optional

TICKS_PER_REV = 4096
TWO_PI = 2 * math.pi
HOME_TICKS = 2048  # Servo midpoint = neutral/home position

# IoT telemetry joint names (from Greengrass edge component)
IOT_JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

# Isaac Sim USD joint names (from /so101_new_calib/joints/ prims)
JOINT_NAMES = [
    'Rotation',
    'Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Jaw',
]

# Mapping from IoT names to Isaac Sim USD joint names (by index order)
IOT_TO_USD = dict(zip(IOT_JOINT_NAMES, JOINT_NAMES))


def ticks_to_radians(ticks: int) -> float:
    """Convert servo position ticks to radians offset from home.

    Args:
        ticks: Servo position in ticks (0-4096).

    Returns:
        float: Position in radians relative to home. 0 = home (2048 ticks),
               range -π to +π. Isaac Sim joint zero = neutral arm pose.
    """
    return (ticks - HOME_TICKS) * (TWO_PI / TICKS_PER_REV)


def convert_payload(payload: dict) -> Optional[dict]:
    """Convert IoT telemetry payload to joint positions in radians.

    Only extracts positions — velocity and effort are not sent to Isaac Sim
    as they conflict with position-based articulation control.

    Args:
        payload: Raw IoT telemetry payload.

    Returns:
        dict with 'positions' (USD joint name -> radians) and 'timestamp',
        or None if malformed.
    """
    joints = payload.get('joints')
    if not joints:
        return None

    result = {
        'positions': {},
        'timestamp': payload.get('timestamp', 0),
    }

    for iot_name, usd_name in IOT_TO_USD.items():
        joint = joints.get(iot_name)
        if joint is None:
            continue
        result['positions'][usd_name] = ticks_to_radians(joint.get('position', 0))

    return result
