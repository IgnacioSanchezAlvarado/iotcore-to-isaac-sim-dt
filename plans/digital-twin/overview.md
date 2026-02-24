# Implementation Plan — Real-Time Digital Twin

## Phases & Dependencies

```
Phase 1: Project Setup & CDK Infrastructure
    │
    ├──► Phase 2: IoT MQTT Subscriber (Python backend)
    │        │
    │        └──► Phase 3: ROS2 Bridge + Isaac Sim Integration
    │                 │
    │                 └──► Phase 4: End-to-End Testing & Polish
    │
    └──► Phase 2b: EC2 Bootstrap Script (can parallel with Phase 2)
```

### Parallel Opportunities

- **Wave 1**: Phase 1 (CDK stack) — must complete first
- **Wave 2**: Phase 2 (MQTT subscriber) + Phase 2b (UserData bootstrap script) — independent of each other
- **Wave 3**: Phase 3 (ROS2 bridge + Isaac Sim) — depends on Phase 2 + 2b
- **Wave 4**: Phase 4 (testing, monitoring, docs) — depends on Phase 3

## Phase Files

| File | Phase | Description |
|------|-------|-------------|
| `01-project-setup.md` | 1 | CDK project, VPC, EC2, IAM, security group |
| `02-mqtt-subscriber.md` | 2 | Python IoT Core MQTT5 subscriber with SigV4 |
| `02b-ec2-bootstrap.md` | 2b | UserData script: ROS2, Python deps, USD model |
| `03-ros2-bridge-isaacsim.md` | 3 | ROS2 joint state publisher + Isaac Sim scene |
| `04-testing-polish.md` | 4 | End-to-end test, monitoring, docs |

## Key Architecture Decisions (from ARCHITECTURE.md)

1. **IoT Auth**: MQTT5/WebSocket with IAM/SigV4 (no X.509 certs)
2. **Bridge**: Custom Python node (MQTT subscriber → ROS2 publisher)
3. **Isaac Sim**: ROS2 action graph in USD model (fallback: Python API)
4. **Networking**: Public subnet + locked-down security group
5. **Visualization**: NICE DCV on port 8443
