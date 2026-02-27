"""Joint data conversion utilities for LeRobot SO-101 telemetry.

Converts servo tick positions (0-4096) to radians and extracts joint data
from IoT Core telemetry payloads.
"""

import logging
import math
from typing import Optional

logger = logging.getLogger(__name__)

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

# Per-joint offset in radians to align USD model zero with physical servo zero
JOINT_OFFSETS = {
    'Wrist_Roll': math.pi / 2,  # Wrist roll: USD zero is vertical, physical zero is horizontal
}

# Per-joint scale factor (applied before offset) to match USD model range
JOINT_SCALES = {
    'Jaw': 0.25,  # Gripper: USD jaw range is smaller than raw degree range
}

# Format detection flag for one-time logging
_format_detected = False


def ticks_to_radians(ticks: int) -> float:
    """Convert servo position ticks to radians offset from home.

    Args:
        ticks: Servo position in ticks (0-4096).

    Returns:
        float: Position in radians relative to home. 0 = home (2048 ticks),
               range -π to +π. Isaac Sim joint zero = neutral arm pose.
    """
    return (ticks - HOME_TICKS) * (TWO_PI / TICKS_PER_REV)


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians.

    Args:
        degrees: Angle in degrees.

    Returns:
        float: Angle in radians.
    """
    return degrees * (math.pi / 180)


def _detect_format(joints: dict) -> str:
    """Detect whether joint positions are in degrees or ticks.

    Auto-detection logic: If ANY position value is negative OR is a float
    with a fractional part, treat as degrees. Otherwise treat as ticks.

    Args:
        joints: Dictionary of joint data from IoT payload.

    Returns:
        str: "degrees" or "ticks"
    """
    for joint_data in joints.values():
        if not isinstance(joint_data, dict):
            continue
        position = joint_data.get('position')
        if position is None:
            continue
        # Check if negative (ticks are 0-4096, so negative means degrees)
        if position < 0:
            return "degrees"
        # Check if float with fractional part (ticks are integers)
        if isinstance(position, float) and position != int(position):
            return "degrees"
    return "ticks"


def convert_payload(payload: dict) -> Optional[dict]:
    """Convert IoT telemetry payload to joint positions in radians.

    Automatically detects whether positions are in degrees or ticks and converts
    accordingly. Only extracts positions — velocity and effort are not sent to
    Isaac Sim as they conflict with position-based articulation control.

    Args:
        payload: Raw IoT telemetry payload.

    Returns:
        dict with 'positions' (USD joint name -> radians) and 'timestamp',
        or None if malformed.
    """
    global _format_detected

    joints = payload.get('joints')
    if not joints:
        return None

    # Detect format and log once
    format_type = _detect_format(joints)
    if not _format_detected:
        logger.info(f"Telemetry format detected: {format_type}")
        _format_detected = True

    # Choose conversion function based on detected format
    convert_fn = degrees_to_radians if format_type == "degrees" else ticks_to_radians

    result = {
        'positions': {},
        'timestamp': payload.get('timestamp', 0),
    }

    for iot_name, usd_name in IOT_TO_USD.items():
        joint = joints.get(iot_name)
        if joint is None:
            continue
        radians = convert_fn(joint.get('position', 0))
        radians = radians * JOINT_SCALES.get(usd_name, 1.0) + JOINT_OFFSETS.get(usd_name, 0.0)
        result['positions'][usd_name] = radians

    return result
