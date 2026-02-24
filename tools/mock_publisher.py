#!/usr/bin/env python3
"""Mock telemetry publisher for testing the digital twin.

Publishes synthetic servo data to IoT Core, mimicking the
Greengrass edge component's mock mode (GG_MOCK_MODE=true).

Mock data generates realistic robot arm movements:
- Coordinated joint motion simulating pick-and-place task (~10s cycle)
- Velocity correlated with position changes
- Load varies by joint (higher for shoulder/elbow, lower for wrist/gripper)
- Realistic amplitude and frequency per joint

Usage:
    python mock_publisher.py                    # 10Hz, default region
    python mock_publisher.py --rate 5           # 5Hz
    python mock_publisher.py --region us-west-2 # different region
"""

import argparse
import json
import math
import random
import sys
import time

import boto3

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

# Realistic joint motion parameters
# Each tuple: (base_offset, amplitude, frequency, phase_offset)
JOINT_PARAMS = {
    'shoulder_pan':   (0,    150, 0.15, 0.0),        # Slow base rotation
    'shoulder_lift':  (0,    200, 0.20, math.pi/4),  # Primary motion joint
    'elbow_flex':     (200,  250, 0.25, math.pi/2),  # Slightly bent, larger motion
    'wrist_flex':     (0,    100, 0.30, math.pi/3),  # Small wrist movement
    'wrist_roll':     (0,     80, 0.20, math.pi/6),  # Very small roll
    'gripper':        (-248, 200, 0.10, 0.0),        # Mostly closed (base at 1800)
}

# Load characteristics (higher for joints bearing more weight)
JOINT_LOAD_BASE = {
    'shoulder_pan':   25,  # Base rotation, moderate load
    'shoulder_lift':  35,  # Lifts entire arm, high load
    'elbow_flex':     30,  # Lifts forearm, high load
    'wrist_flex':     15,  # Lower load
    'wrist_roll':     10,  # Minimal load
    'gripper':        20,  # Variable depending on grip
}

# Global task cycle period (seconds) - simulates repetitive pick-and-place
TASK_PERIOD = 10.0

# Store previous positions for velocity calculation
_prev_positions = {}


def generate_payload(t: float, device_id: str = 'arm-001') -> dict:
    """Generate a single mock telemetry payload with realistic motion."""
    global _prev_positions

    # Global task modulator - slow sinusoid representing pick-and-place cycle
    task_phase = math.sin(2 * math.pi * t / TASK_PERIOD)

    joints = {}
    for name in JOINT_NAMES:
        base_offset, amplitude, frequency, phase_offset = JOINT_PARAMS[name]

        # Position with global task modulation
        # Task modulator scales amplitude slightly (0.7 to 1.0) to simulate coordinated motion
        task_scale = 0.85 + 0.15 * (task_phase * 0.5 + 0.5)
        raw_position = 2048 + base_offset + int(task_scale * amplitude * math.sin(2 * math.pi * frequency * t + phase_offset))

        # Gripper uses different pattern - mostly closed with occasional opening
        if name == 'gripper':
            # Triangle wave that spends most time at one extreme
            gripper_cycle = (t * frequency) % 1.0
            if gripper_cycle < 0.7:
                gripper_pos = 0.0  # Closed
            elif gripper_cycle < 0.85:
                gripper_pos = (gripper_cycle - 0.7) / 0.15  # Opening
            else:
                gripper_pos = 1.0 - (gripper_cycle - 0.85) / 0.15  # Closing
            raw_position = 2048 + base_offset + int(amplitude * gripper_pos)

        # Clamp to valid servo range
        position = max(0, min(4095, raw_position))

        # Velocity - correlated with position change (approximate derivative)
        if name in _prev_positions:
            # Calculate velocity in ticks per time unit, scaled to reasonable range
            delta = position - _prev_positions[name]
            velocity = int(delta * 2.0)  # Scale factor for readability
            # Add small noise
            velocity += random.randint(-2, 2)
            # Clamp to servo velocity range
            velocity = max(-50, min(50, velocity))
        else:
            velocity = 0

        _prev_positions[name] = position

        # Load - base load per joint + variation based on motion
        base_load = JOINT_LOAD_BASE[name]
        # Load increases with motion magnitude
        motion_load = int(abs(velocity) * 0.5)
        load = base_load + motion_load + random.randint(-3, 3)
        load = max(0, min(100, load))

        # Temperature - slight variation, slightly higher under load
        temp_base = 32 + int(load * 0.1)
        temp = temp_base + random.randint(-2, 2)
        temp = max(25, min(45, temp))

        # Current - correlates with load
        current_base = 100 + int(load * 3)
        current = current_base + random.randint(-20, 20)
        current = max(50, min(400, current))

        joints[name] = {
            'position': position,
            'velocity': velocity,
            'load': load,
            'temp': temp,
            'current': current,
        }

    return {
        'device_id': device_id,
        'timestamp': int(time.time() * 1000),
        'joints': joints,
    }


def main():
    parser = argparse.ArgumentParser(description='Mock telemetry publisher')
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz (default: 10)')
    parser.add_argument('--region', type=str, default='us-east-1', help='AWS region')
    parser.add_argument('--topic', type=str, default='dt/lerobot/arm-001/telemetry', help='MQTT topic')
    parser.add_argument('--device-id', type=str, default='arm-001', help='Device ID')
    parser.add_argument('--duration', type=float, default=0, help='Duration in seconds (0 = infinite)')
    args = parser.parse_args()

    client = boto3.client('iot-data', region_name=args.region)
    interval = 1.0 / args.rate

    print(f"Publishing mock telemetry to {args.topic} at {args.rate} Hz")
    print(f"Region: {args.region}, Device: {args.device_id}")
    print("Press Ctrl+C to stop\n")

    t = 0.0
    count = 0
    start_time = time.time()

    try:
        while True:
            payload = generate_payload(t, args.device_id)
            try:
                client.publish(
                    topic=args.topic,
                    qos=0,
                    payload=json.dumps(payload),
                )
            except Exception as e:
                print(f"ERROR: Failed to publish: {e}")
                print("Check AWS credentials and IoT permissions.")
                sys.exit(1)
            count += 1
            t += interval

            if count % int(args.rate * 10) == 0:
                elapsed = time.time() - start_time
                actual_rate = count / elapsed if elapsed > 0 else 0
                print(f"Published {count} messages ({actual_rate:.1f} msg/s)")

            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                break

            time.sleep(interval)

    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start_time
    print(f"\nDone. Published {count} messages in {elapsed:.1f}s ({count/elapsed:.1f} msg/s)")


if __name__ == '__main__':
    main()
