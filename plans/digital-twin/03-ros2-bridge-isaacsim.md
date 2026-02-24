# Phase 3: ROS2 Bridge + Isaac Sim Integration

## Goal

Connect the MQTT subscriber to Isaac Sim via ROS2. A single Python process subscribes to IoT Core MQTT, converts telemetry to `sensor_msgs/JointState`, publishes to `/joint_states`, and Isaac Sim's USD action graph drives the robot model.

## Dependencies

- Phase 2 (MQTT subscriber and TelemetryBuffer)
- Phase 2b (ROS2 installed, USD model downloaded)

## Architecture

```
MQTT (IoT Core) → Python Bridge → ROS2 /joint_states → Isaac Sim USD Action Graph
```

The bridge is a single Python process that:
1. Runs the MQTT5 subscriber (from Phase 2) in a background thread
2. Runs a ROS2 node that publishes `JointState` messages
3. Reads from the TelemetryBuffer and publishes at received rate

## Tasks

### 3.1 ROS2 Joint State Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self, telemetry_buffer):
        super().__init__('dt_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.buffer = telemetry_buffer
        # Timer at 20Hz (2x telemetry rate) to avoid missing updates
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        self.last_timestamp = 0

    def publish_joint_state(self):
        latest = self.buffer.get_latest()
        if latest is None:
            return
        if latest.get('timestamp', 0) == self.last_timestamp:
            return  # No new data

        self.last_timestamp = latest.get('timestamp', 0)
        joints = latest.get('joints', {})

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'gripper'
        ]
        msg.position = [
            ticks_to_radians(joints[name]['position'])
            for name in msg.name
        ]
        msg.velocity = [
            ticks_to_radians(joints[name]['velocity'])
            for name in msg.name
        ]
        msg.effort = [
            float(joints[name]['load'])
            for name in msg.name
        ]

        self.publisher.publish(msg)
```

### 3.2 Bridge Main Entry Point

```python
# bridge.py — Single process combining MQTT subscriber + ROS2 publisher
import threading
import rclpy

def main():
    # 1. Start MQTT subscriber (background thread)
    buffer = TelemetryBuffer()
    mqtt_thread = threading.Thread(target=start_mqtt_subscriber, args=(buffer,), daemon=True)
    mqtt_thread.start()

    # 2. Start ROS2 node (main thread)
    rclpy.init()
    node = JointStatePublisher(buffer)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.3 Isaac Sim Scene Loading (Option A — ROS2 Action Graph)

If the USD model's ROS2 action graph subscribes to `/joint_states` and joint names match:

- Load the scene: Open Isaac Sim GUI or run headless
- File → Open → `/opt/digital-twin/assets/SO-ARM101-USD.usd`
- Press Play — the action graph should auto-subscribe to `/joint_states`
- The bridge publishes joint states; Isaac Sim renders the robot

### 3.4 Isaac Sim Python Script (Option B — Fallback)

If the action graph doesn't work or joint names don't match:

```python
# isaac_driver.py — Run inside Isaac Sim's Python environment
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np

# Load scene
world = World()
world.scene.add_default_ground_plane()
# Add robot from USD
robot = world.scene.add(Articulation(
    prim_path="/World/SO_ARM101",  # Verify actual path in USD
    name="so_arm101"
))
world.reset()

# Main loop — read from shared memory / file / socket
while True:
    world.step(render=True)
    # Read latest joint positions from buffer/file
    positions = get_latest_positions()  # [rad1, rad2, ..., rad6]
    if positions:
        robot.set_joint_positions(np.array(positions))
```

### 3.5 Joint Name Mapping

The IoT payload uses these names (must match USD action graph expectations):

| IoT Name         | Motor ID | Expected ROS2 Name | USD Prim Name (TBD) |
|------------------|----------|--------------------|--------------------|
| `shoulder_pan`   | 1        | `shoulder_pan`     | Verify after AMI boot |
| `shoulder_lift`  | 2        | `shoulder_lift`    | Verify after AMI boot |
| `elbow_flex`     | 3        | `elbow_flex`       | Verify after AMI boot |
| `wrist_flex`     | 4        | `wrist_flex`       | Verify after AMI boot |
| `wrist_roll`     | 5        | `wrist_roll`       | Verify after AMI boot |
| `gripper`        | 6        | `gripper`          | Verify after AMI boot |

**Action item**: After launching the AMI, inspect the USD file to find exact joint prim names. If they differ from the IoT names, add a mapping dict in the bridge.

### 3.6 Verify ROS2 Communication

```bash
# Terminal 1: Run the bridge
source /opt/ros/jazzy/setup.bash
python3 /opt/digital-twin/backend/bridge.py

# Terminal 2: Verify ROS2 messages
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joint_states

# Terminal 3: Check message rate
ros2 topic hz /joint_states
# Should show ~10Hz
```

## Acceptance Criteria

- [ ] Bridge process starts and connects to both IoT Core and ROS2
- [ ] `ros2 topic echo /joint_states` shows correct joint data
- [ ] `ros2 topic hz /joint_states` shows ~10Hz
- [ ] Position values are in radians (not raw ticks)
- [ ] Isaac Sim robot model moves in sync with telemetry
- [ ] Handles connection drops gracefully (reconnect, resume)
