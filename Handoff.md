# LeRobot IoT Core — Project Handoff

This document describes what the **lerobot-iotcore** project delivers: a working edge-to-cloud telemetry pipeline for the LeRobot SO-101 robotic arm. The project is complete and stable. Any downstream consumer (digital twin, dashboard, analytics) can subscribe to the IoT Core MQTT topic described below and start receiving live joint data immediately.

---

## What This Project Does

A Greengrass v2 component runs on a robot host with a connected SO-101 arm. It:

- Reads 5 sensor registers from all 6 servos via serial bus at 10Hz
- Publishes a JSON telemetry message to AWS IoT Core via MQTT (X.509 mutual TLS)
- Simultaneously publishes to local ROS2 Jazzy topics (`/joint_states`, `/servo_diagnostics`)
- Gracefully degrades to IoT Core-only if ROS2 is not installed on the host
- Supports mock mode for generating synthetic data without hardware

**Repository**: `lerobot-iotcore`
**Component**: `com.lerobot.telemetry` v2.0.0
**Runtime**: Python 3.12, ROS2 Jazzy, Greengrass Nucleus 2.14.2
**Infrastructure**: AWS CDK (TypeScript) — IoT Thing, IoT Policy, Greengrass Component + Deployment, IAM roles

---

## Data Available on IoT Core

### MQTT Topic

```
dt/lerobot/{device-id}/telemetry
```

Current device: `dt/lerobot/arm-001/telemetry`

- **QoS**: 0 (at most once)
- **Publish rate**: 10Hz (configurable via `pollingRateHz` in config.json)
- **Protocol**: MQTT over TLS 1.2, X.509 mutual auth via Greengrass
- **Direction**: Edge → Cloud (publish only, no subscriptions from cloud to edge currently)

### JSON Payload

Every message on the topic has this exact structure:

```json
{
  "device_id": "arm-001",
  "timestamp": 1706000000000,
  "joints": {
    "shoulder_pan":  {"position": 2048, "velocity": 0, "load": 12, "temp": 35, "current": 150},
    "shoulder_lift": {"position": 1800, "velocity": 5, "load": 45, "temp": 38, "current": 280},
    "elbow_flex":    {"position": 2200, "velocity": 3, "load": 30, "temp": 36, "current": 200},
    "wrist_flex":    {"position": 2100, "velocity": 0, "load": 8,  "temp": 33, "current": 120},
    "wrist_roll":    {"position": 2048, "velocity": 0, "load": 5,  "temp": 32, "current": 100},
    "gripper":       {"position": 1500, "velocity": 0, "load": 15, "temp": 34, "current": 160}
  }
}
```

### Field Reference

| Field | Type | Units / Range | Description |
|-------|------|---------------|-------------|
| `device_id` | string | e.g. `"arm-001"` | Device identifier, set in config.json |
| `timestamp` | integer | Epoch milliseconds (UTC) | `int(time.time() * 1000)` on the edge device |
| `joints` | object | 6 keys | One entry per joint (always all 6 present) |
| `position` | integer | 0–4096 ticks (12-bit encoder) | Raw servo position |
| `velocity` | integer | ticks/second (signed) | Raw servo velocity |
| `load` | integer | 0–1023 (10-bit ADC) | Raw servo load |
| `temp` | integer | Degrees Celsius | Servo temperature |
| `current` | integer | Milliamps | Servo current draw |

### Conversion Formulas

```python
# Position: servo ticks → radians
radians = position_ticks * (2 * math.pi / 4096)

# Velocity: servo ticks/s → rad/s
rad_per_sec = velocity_ticks * (2 * math.pi / 4096)
```

Key reference points:
- **0 ticks** = 0 radians (servo minimum)
- **2048 ticks** = π radians (mid-range, typical "home" position)
- **4096 ticks** = 2π radians (servo maximum)

---

## Joint Mapping

6 joints, always in this order (matches motor ID order 1–6):

| Joint Name | Motor ID | Physical DOF | Description |
|------------|----------|--------------|-------------|
| `shoulder_pan` | 1 | Z-axis rotation | Base rotation (yaw) |
| `shoulder_lift` | 2 | Y-axis rotation | Shoulder pitch |
| `elbow_flex` | 3 | Y-axis rotation | Elbow pitch |
| `wrist_flex` | 4 | Y-axis rotation | Wrist pitch |
| `wrist_roll` | 5 | X-axis rotation | Wrist rotation (roll) |
| `gripper` | 6 | Parallel jaw | Gripper open/close |

---

## ROS2 Topics (Local on Robot Host)

These are available on the robot host's local DDS network (default domain 0). They are **not** bridged to the cloud — IoT Core MQTT is the cloud path.

**`/joint_states`** — `sensor_msgs/msg/JointState`
- `header.frame_id`: `"base_link"`
- `name`: `["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]`
- `position`: 6 floats (radians, converted from ticks)
- `velocity`: 6 floats (rad/s, converted from ticks)
- `effort`: 6 floats (raw load value cast to float)

**`/servo_diagnostics`** — `diagnostic_msgs/msg/DiagnosticArray`
- One `DiagnosticStatus` per joint, named `servo/{joint_name}`
- Key-value pairs: `temperature` (°C), `current` (mA)
- Level: `OK` normally, `WARN` if temp > 50°C

---

## Robot Hardware Specifications

### SO-101 Arm

| Property | Value |
|----------|-------|
| Degrees of Freedom | 6 (5 revolute + 1 gripper) |
| Servo Model | Feetech STS3215 |
| Serial Protocol | Feetech SCS (similar to Dynamixel), TTL, 1,000,000 baud |
| Power | 12V DC |
| Registers Read | Present_Position, Present_Velocity, Present_Load, Present_Temperature, Present_Current |

### Servo Specs (Feetech STS3215)

| Property | Value |
|----------|-------|
| Position Resolution | 4096 ticks (12-bit) |
| Position Range | 0–4096 (0–360°) |
| Velocity Range | -1023 to +1023 ticks/sec |
| Temperature | Operating -5 to +70°C, software warning at >50°C |
| Load Sensor | 10-bit ADC (0–1023) |
| Current Sensor | 1mA resolution |

### Joint Ranges (Estimated)

| Joint | Type | Approx. Range | Notes |
|-------|------|---------------|-------|
| shoulder_pan | Revolute | 0–360° | Continuous rotation |
| shoulder_lift | Revolute | ~90–270° | Limited by arm geometry |
| elbow_flex | Revolute | ~90–270° | Limited by arm geometry |
| wrist_flex | Revolute | ~90–270° | Limited by arm geometry |
| wrist_roll | Revolute | 0–360° | Continuous rotation |
| gripper | Prismatic | 0–50mm | Modeled as revolute in code (same ticks-to-radians conversion) |

Actual joint limits should be confirmed from the LeRobot documentation or discovered empirically.

---

## Deployed Architecture

```
Robot Host (Edge)                        AWS Cloud
┌─────────────────────────────────┐
│  LeRobot SO-101 Arm             │
│  (6x Feetech STS3215 servos)   │
│         │ Serial USB, 1Mbps     │
│         v                       │
│  Greengrass Component           │
│  com.lerobot.telemetry v2.0.0  │
│  (Python 3.12, ROS2 Jazzy)     │
│         │                       │
│    ┌────┴────────┐              │
│    │             │              │
│    v             v              │
│  Greengrass    ROS2 DDS         │
│  IPC           (local only)     │
│    │           /joint_states    │
│    │           /servo_diag...   │
│    v                            │
│  Greengrass Nucleus             │
│  (MQTT client)                  │
└────┬────────────────────────────┘
     │ MQTT over TLS 1.2
     │ X.509 mutual auth
     v
┌────────────────────────────┐
│  AWS IoT Core              │
│                            │
│  Topic:                    │
│  dt/lerobot/arm-001/       │
│  telemetry                 │
│                            │
│  (No IoT Rules configured  │
│   — downstream routing is  │
│   left to the consumer)    │
└────────────────────────────┘
```

### AWS Resources Deployed (via CDK)

| Resource | Name / Identifier |
|----------|-------------------|
| IoT Thing | `lerobot-arm-001` |
| IoT Policy | `lerobot-arm-001-policy` |
| IAM Role | `lerobot-arm-001-token-exchange-role` |
| IoT Role Alias | `lerobot-arm-001-role-alias` |
| Greengrass Component | `com.lerobot.telemetry` v2.0.0 |
| Greengrass Deployment | `lerobot-arm-001-initial-deployment` |

IoT Policy permissions (least privilege):
- `iot:Connect` — scoped to thing client ID
- `iot:Publish` — scoped to `dt/lerobot/arm-001/*`
- `iot:Subscribe` — scoped to `dt/lerobot/arm-001/*`
- `iot:Receive` — scoped to `dt/lerobot/arm-001/*`

---

## Technical Notes

### Timing and Latency

- Timestamp is edge device wall clock: `int(time.time() * 1000)`
- No NTP enforcement on the edge — clock may drift
- Serial read cycle for all 6 motors: ~30–60ms
- Full publish cycle at 10Hz: ~50–70ms per iteration
- Remaining time in the 100ms interval is sleep

### Mock Mode

The component can generate synthetic telemetry without a physical arm. Set `GG_MOCK_MODE=true` in the Greengrass component configuration (or `--mock` on CLI).

Mock data characteristics:
- `position = 2048 + int(500 * sin(t * freq + phase))`
- `freq = 0.5 + i * 0.1` per joint (0.5Hz for shoulder_pan through 1.0Hz for gripper)
- `phase = i * π/3` per joint (each offset by 60°)
- Velocity: random -10 to 10, Load: random 5–50, Temp: random 30–40°C, Current: random 80–300mA

### Configuration

All settings are in `config.json` at the project root. The CDK stack and Greengrass component both read from it:

```json
{
  "project":    { "name": "lerobot-iot", "environment": "dev" },
  "iot":        { "deviceId": "arm-001", "topicPrefix": "dt/lerobot" },
  "component":  { "name": "com.lerobot.telemetry", "version": "2.0.0", "pollingRateHz": 10 },
  "ros2":       { "nodeName": "lerobot_telemetry", "jointStatesTopic": "/joint_states",
                  "servoDiagnosticsTopic": "/servo_diagnostics", "distro": "jazzy" },
  "greengrass": { "nucleusVersion": "2.14.2" }
}
```

### Security

- X.509 mutual TLS via Greengrass provisioning (no passwords or API keys)
- IoT policy is least-privilege, scoped to the specific device ID
- Greengrass IPC uses a local Unix socket (`/run/greengrass_ipc.sock`)
- Token exchange role grants only CloudWatch Logs + S3 read (CDK asset bucket)

---

## References

- [LeRobot Repository](https://github.com/huggingface/lerobot)
- [SO-101 Documentation](https://github.com/huggingface/lerobot/tree/main/examples/10_use_so101)
- [Feetech STS3215 Datasheet](https://www.feetechrc.com/products/sts3215-serial-bus-servo)
- [AWS IoT Core MQTT](https://docs.aws.amazon.com/iot/latest/developerguide/mqtt.html)
- [AWS IoT Greengrass v2](https://docs.aws.amazon.com/greengrass/v2/developerguide/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)

---

**Source Project**: lerobot-iotcore (commit: e53f4e5)
**Last Updated**: 2026-02-19
