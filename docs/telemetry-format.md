# Telemetry Data Format and Conversion Pipeline

Reference documentation for the digital twin telemetry data flow from IoT Core to Isaac Sim.

## 1. IoT Core Telemetry Payload

The Greengrass edge component publishes JSON to MQTT topic `dt/lerobot/<device-id>/telemetry` at 10Hz.

### Example Payload

```json
{
  "device_id": "arm-001",
  "timestamp": 1706000000000,
  "joints": {
    "shoulder_pan":  { "position": 2.01,   "velocity": 0.0,  "load": 20.0, "temp": 31, "current": 0 },
    "shoulder_lift": { "position": -98.89, "velocity": 0.0,  "load": 0.0,  "temp": 29, "current": 0 },
    "elbow_flex":    { "position": 97.27,  "velocity": 0.0,  "load": 80.0, "temp": 25, "current": 3 },
    "wrist_flex":    { "position": 63.70,  "velocity": 0.0,  "load": 1080, "temp": 29, "current": 1 },
    "wrist_roll":    { "position": -3.44,  "velocity": 0.0,  "load": 36.0, "temp": 29, "current": 1 },
    "gripper":       { "position": 4.83,   "velocity": 0.0,  "load": 1048, "temp": 33, "current": 0 }
  }
}
```

### Field Definitions

- **`device_id`**: Robot identifier (e.g., `"arm-001"`)
- **`timestamp`**: Unix epoch in milliseconds
- **`joints`**: Object containing 6 joints, each with:
  - **`position`**: Joint angle in degrees (float). 0° = home/neutral position. Can be negative.
  - **`velocity`**: Joint velocity in degrees/second (float, signed)
  - **`load`**: Raw servo load value
  - **`temp`**: Temperature in Celsius
  - **`current`**: Current draw in mA

The 6 joints correspond to motor IDs 1–6 on the SO-101 robotic arm.

## 2. Joint Name Mapping

The IoT telemetry uses descriptive joint names. Isaac Sim's USD model uses different names for the same physical joints.

| Motor ID | IoT Name | USD Joint Name | Description |
|----------|----------|----------------|-------------|
| 1 | shoulder_pan | Rotation | Base rotation |
| 2 | shoulder_lift | Pitch | Shoulder pitch |
| 3 | elbow_flex | Elbow | Elbow bend |
| 4 | wrist_flex | Wrist_Pitch | Wrist pitch |
| 5 | wrist_roll | Wrist_Roll | Wrist rotation |
| 6 | gripper | Jaw | Gripper open/close |

This mapping is implemented in `backend/joint_converter.py` via the `IOT_TO_USD` dictionary.

## 3. Position Conversion (Degrees to Radians)

The Greengrass edge component publishes positions in **degrees** (0° = home). The converter auto-detects the format and converts to radians for Isaac Sim.

### Conversion Formula (degrees)

```
radians = degrees × (π / 180)
```

### Legacy Format (ticks)

Older versions of the edge component may publish raw servo ticks (0–4096 integers). The converter auto-detects this and uses:

```
radians = (ticks - 2048) × (2π / 4096)
```

Detection logic: if any position value is negative or has a fractional part → degrees. Otherwise → ticks.

**Note**: Only position values are sent to the Isaac Sim ArticulationController. Velocity and effort are not used — they conflict with position-based control.

## 5. Data Pipeline

```mermaid
sequenceDiagram
    participant IoT as IoT Core
    participant Sub as subscriber.py
    participant Buf as TelemetryBuffer
    participant Conv as joint_converter.py
    participant Pub as ros2_publisher.py
    participant Sim as Isaac Sim

    IoT->>Sub: MQTT message (JSON)
    Sub->>Conv: Raw payload
    Conv->>Buf: Converted payload (radians)
    Pub->>Buf: Poll for latest
    Buf->>Pub: Positions + velocities
    Pub->>Sim: JointState msg on /isaac_joint_command
```

### Pipeline Steps

1. **`subscriber.py`** connects to IoT Core via MQTT5 WebSocket with SigV4/IAM auth
2. Each incoming message is passed to **`joint_converter.convert_payload()`** which maps joint names and converts ticks to radians
3. The converted data is stored in a thread-safe **`TelemetryBuffer`** (single-slot, latest-value-wins)
4. **`ros2_publisher.py`** polls the buffer at 20Hz and publishes `sensor_msgs/JointState` messages to `/isaac_joint_command`
5. Isaac Sim's OmniGraph action graph subscribes to this topic and drives the ArticulationController

## 6. ROS2 JointState Message

The final ROS2 message format that reaches Isaac Sim:

```
header:
  stamp: <current_time>
  frame_id: "base_link"
name: ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
position: [0.0, 0.307, 0.384, 0.0, 0.0, -0.380]
velocity: [0.0, 0.018, -0.008, 0.005, 0.0, 0.0]
effort: [25.0, 35.0, 30.0, 15.0, 10.0, 20.0]
```

### Field Usage

- **`name`** and **`position`**: Used by the ArticulationController to drive joint angles
- **`velocity`** and **`effort`**: Published for monitoring but not wired to the simulation (effort stores the load percentage)

The positions array order matches the names array and follows the motor ID sequence (1–6).
