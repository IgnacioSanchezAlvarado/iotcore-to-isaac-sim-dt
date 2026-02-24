# Phase 4: End-to-End Testing & Polish

## Goal

Validate the full pipeline from IoT Core to Isaac Sim visualization. Add monitoring, error handling, and documentation.

## Dependencies

- Phase 3 (bridge + Isaac Sim integration working)

## Tasks

### 4.1 Mock Telemetry Publisher (for testing without physical robot)

Create a local Python script that publishes mock data to IoT Core, mimicking the Greengrass edge component's mock mode:

```python
# tools/mock_publisher.py
import json, math, time, boto3

client = boto3.client('iot-data', region_name='us-east-1')
TOPIC = 'dt/lerobot/arm-001/telemetry'
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']

t = 0
while True:
    joints = {}
    for i, name in enumerate(JOINT_NAMES):
        freq = 0.5 + i * 0.1
        phase = i * math.pi / 3
        joints[name] = {
            'position': 2048 + int(500 * math.sin(t * freq + phase)),
            'velocity': random.randint(-10, 10),
            'load': random.randint(5, 50),
            'temp': random.randint(30, 40),
            'current': random.randint(80, 300),
        }
    payload = {
        'device_id': 'arm-001',
        'timestamp': int(time.time() * 1000),
        'joints': joints,
    }
    client.publish(topic=TOPIC, qos=0, payload=json.dumps(payload))
    t += 0.1
    time.sleep(0.1)  # 10Hz
```

### 4.2 End-to-End Test Checklist

1. Start mock publisher (locally or on another EC2)
2. Verify MQTT messages arrive (`ros2 topic echo /joint_states`)
3. Confirm Isaac Sim robot arm moves smoothly
4. Verify ~10Hz update rate (`ros2 topic hz /joint_states`)
5. Kill mock publisher — verify bridge handles disconnect gracefully
6. Restart mock publisher — verify bridge resumes
7. Kill bridge process — verify systemd restarts it
8. Test with actual robot (if available) via Greengrass mock mode

### 4.3 Monitoring & Health

Add to the bridge process:

- **Log message rate** every 30 seconds (expected: ~10 msg/s)
- **Log latency** (wall clock − message timestamp), warn if >500ms
- **Watchdog**: Log warning if no message for >500ms (5 missed cycles)
- **CloudWatch metrics** (optional): message_count, latency_ms, dropped_messages

### 4.4 Error Handling

- MQTT disconnect → auto-reconnect (handled by SDK)
- Malformed JSON → log warning, skip message
- Missing joint in payload → log warning, publish partial update
- Isaac Sim crash → bridge continues running, ROS2 messages queue
- Bridge crash → systemd restarts within 5 seconds

### 4.5 Documentation

Update README.md with:
1. What it does (2-3 sentences)
2. Architecture diagram (simplified)
3. Deploy: `cd infra && npx cdk deploy`
4. Connect: DCV URL from CDK output
5. Test: Run mock publisher, watch robot move
6. Clean up: `cd infra && npx cdk destroy`

## Acceptance Criteria

- [ ] Mock publisher generates realistic synthetic telemetry
- [ ] Full pipeline works: mock publisher → IoT Core → bridge → ROS2 → Isaac Sim
- [ ] Robot arm in Isaac Sim moves smoothly at 10Hz
- [ ] Bridge recovers from MQTT disconnects automatically
- [ ] Systemd restarts bridge on crash
- [ ] Logging shows message rate and latency
- [ ] README has deploy/test/cleanup instructions
