# Architecture — Real-Time Digital Twin (Isaac Sim)

## Overview

A real-time digital twin of a LeRobot SO-101 robotic arm using NVIDIA Isaac Sim on AWS. An EC2 G6e.2xlarge instance runs Isaac Sim, subscribes to 10Hz joint telemetry from AWS IoT Core via MQTT/WebSocket, converts ticks to radians, publishes ROS2 `/joint_states`, and Isaac Sim's USD action graph drives the simulated robot.

## Architecture Diagram

```
Edge (Existing — do not modify)             AWS Cloud
┌───────────────────────┐
│ LeRobot SO-101        │
│ 6x Feetech STS3215   │
│       │ Serial/USB    │
│       v               │
│ Greengrass v2         │
│ com.lerobot.telemetry │
│ (10Hz, QoS 0)        │
└───────┬───────────────┘
        │ MQTT/TLS (X.509)
        v
┌──────────────────────────────────────────────────────────┐
│                    AWS IoT Core                          │
│  Topic: dt/lerobot/arm-001/telemetry                     │
└───────────────────────┬──────────────────────────────────┘
                        │ MQTT over WebSocket (SigV4/IAM)
                        v
┌──────────────────────────────────────────────────────────┐
│  VPC (10.0.0.0/16) — Single AZ                           │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Public Subnet (10.0.1.0/24)                       │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │  EC2 G6e.2xlarge (Isaac Sim AMI)             │  │  │
│  │  │                                              │  │  │
│  │  │  ┌─────────────────────┐                     │  │  │
│  │  │  │ IoT MQTT Subscriber │ Python, awsiotsdk   │  │  │
│  │  │  │ (MQTT5/WebSocket)   │                     │  │  │
│  │  │  └────────┬────────────┘                     │  │  │
│  │  │           │ Ticks → Radians                  │  │  │
│  │  │           v                                  │  │  │
│  │  │  ┌─────────────────────┐                     │  │  │
│  │  │  │ ROS2 Joint State    │ sensor_msgs/        │  │  │
│  │  │  │ Publisher           │ JointState           │  │  │
│  │  │  └────────┬────────────┘                     │  │  │
│  │  │           │ /joint_states (DDS)              │  │  │
│  │  │           v                                  │  │  │
│  │  │  ┌─────────────────────┐                     │  │  │
│  │  │  │ Isaac Sim           │ SO-ARM101-USD.usd   │  │  │
│  │  │  │ (ROS2 Action Graph) │ + physics sim       │  │  │
│  │  │  └────────┬────────────┘                     │  │  │
│  │  │           │ Pixel stream                     │  │  │
│  │  │           v                                  │  │  │
│  │  │  ┌─────────────────────┐                     │  │  │
│  │  │  │ NICE DCV Server     │ Port 8443/HTTPS     │  │  │
│  │  │  └────────┬────────────┘                     │  │  │
│  │  │           │                                  │  │  │
│  │  └───────────┼──────────────────────────────────┘  │  │
│  └──────────────┼─────────────────────────────────────┘  │
│                 │                                         │
│  ┌──────────────┼──────┐                                 │
│  │ Security Group      │                                 │
│  │ IN:  8443/TCP (DCV) │                                 │
│  │      22/TCP (SSH)   │                                 │
│  │ OUT: All            │                                 │
│  └─────────────────────┘                                 │
└──────────────────────────────────────────────────────────┘
        │
        v (HTTPS port 8443)
┌─────────────────┐
│ User's Browser  │
│ or DCV Client   │
└─────────────────┘
```

## Technology Decisions

### 1. IoT Core Auth: MQTT5 over WebSocket with IAM/SigV4

**Chosen over** X.509 device certificates and boto3 HTTP polling.

| Criterion             | SigV4/WebSocket (chosen) | X.509 Certificates       | boto3 HTTP        |
|-----------------------|--------------------------|--------------------------|-------------------|
| CDK complexity        | Low (IAM only)           | High (Custom Resource)   | N/A               |
| Certificate mgmt      | None                     | Full lifecycle           | None              |
| Protocol              | MQTT over WSS (443)      | MQTT direct (8883)       | HTTPS (443)       |
| Real-time capable     | Yes                      | Yes                      | No (100-300ms)    |
| Credential refresh    | Automatic (IMDS)         | N/A                      | Automatic (IMDS)  |
| Reconnection          | SDK built-in             | SDK built-in             | N/A               |

**Rationale**: EC2 already has an IAM instance profile. SigV4 leverages it directly — zero certificate management, no IoT Thing/Policy Custom Resources in CDK, just 4 IAM policy statements. The 24-hour WebSocket connection limit is fine for a POC (SDK auto-reconnects). The `awsiotsdk` library handles SigV4 signing and credential refresh transparently.

**Library**: `awsiotsdk` (AWS IoT Device SDK v2 for Python) — uses `mqtt5_client_builder.websockets_with_default_aws_signing()`.

### 2. MQTT-to-Simulation Bridge: Custom ROS2 Node

**Chosen over** `mqtt_bridge` ROS2 package and direct Isaac Sim extension.

**Rationale**: A single Python process that subscribes to IoT Core MQTT and publishes `sensor_msgs/JointState` to ROS2 `/joint_states` is the simplest approach. The upstream USD model already has a ROS2 action graph that subscribes to `/joint_states`, so we just need to feed it. No custom Isaac Sim extensions needed.

**Pattern**: Thread-safe single-slot buffer. MQTT callback overwrites the latest telemetry; main loop publishes to ROS2 at the received rate. Stale messages are naturally discarded.

### 3. Isaac Sim Integration: ROS2 Action Graph (Primary) with Python API Fallback

**Primary (Option A)**: The USD model from `so-arm101-ros2-bridge` has a pre-configured ROS2 action graph. Publishing to `/joint_states` should drive the robot automatically.

**Fallback (Option B)**: If the ROS2 action graph doesn't work as expected (joint name mismatch, topic mismatch), use Isaac Sim's Python API (`omni.isaac.core`) to set joint positions directly via `ArticulationView.set_joint_positions()`.

**Open question**: The exact joint prim names and ROS2 topic in the USD action graph need to be verified after launching the AMI. First boot should include inspection of the USD file.

### 4. Remote Visualization: NICE DCV

**Rationale**: AWS-native GPU remote desktop, no additional licensing on EC2. H.264 hardware compression via NVIDIA GPU. Better than VNC/RDP for GPU workloads. Port 8443 (HTTPS/TLS).

### 5. Networking: Public Subnet with Locked-Down Security Group

**Chosen over** private subnet + NAT Gateway.

**Rationale**: POC simplicity. The EC2 needs outbound internet for IoT Core (WSS on 443), Isaac Sim licensing (Omniverse), and package updates. A NAT Gateway adds ~$32/month and complexity for no security benefit in a POC with a properly configured security group. Inbound restricted to the user's IP only.

### 6. EC2 Instance: G6e.2xlarge

| Spec      | Value                    |
|-----------|--------------------------|
| vCPUs     | 8                        |
| RAM       | 64 GiB                   |
| GPU       | 1x NVIDIA L40S (48GB)    |
| AMI       | `prodview-bl35herdyozhw` |
| Storage   | 100 GB gp3               |

## Security Design

- **IAM role** on EC2 with least-privilege: `iot:Connect`, `iot:Subscribe`, `iot:Receive` (scoped to `dt/lerobot/arm-001/*`), `iot:DescribeEndpoint`, `ssm:*` (for SSM Session Manager)
- **Security group**: Inbound only 8443 (DCV) and 22 (SSH) from configured client IP CIDR
- **All outbound allowed** (IoT Core, licensing, updates)
- **No public storage or databases**
- **Encrypted in transit**: TLS for MQTT/WebSocket, HTTPS for DCV
- **SSM Session Manager** as SSH alternative (no port 22 needed if preferred)

## CDK Stack (Single Stack)

```
IsaacSimDigitalTwinStack
├── VPC (10.0.0.0/16, 1 AZ, public subnet only)
├── Security Group (8443 + 22 inbound from client IP)
├── IAM Role (IoT Core subscribe + SSM + CloudWatch)
├── EC2 Instance (G6e.2xlarge, Isaac Sim AMI)
│   └── UserData (bootstrap: ROS2, Python deps, backend scripts, USD model)
└── Outputs (Instance ID, DCV URL, IoT endpoint)
```

No IoT-specific CDK constructs needed (no CfnThing, CfnPolicy, CfnCertificate) since we use IAM/SigV4.

## Data Flow (per message, ~100ms cycle)

1. Edge device reads 6 servos via serial → publishes JSON to IoT Core (MQTT/TLS)
2. EC2 subscriber receives message via MQTT5/WebSocket (SigV4 auth)
3. Python parses JSON, converts `position` ticks → radians: `rad = ticks * (2π / 4096)`
4. Publishes `sensor_msgs/JointState` to ROS2 `/joint_states` topic
5. Isaac Sim's ROS2 action graph receives joint state, updates articulation
6. Physics simulation steps, renders frame
7. NICE DCV streams rendered frame to user's client

## Estimated Monthly Cost (us-east-1, POC usage)

| Service          | Usage                       | Cost     |
|------------------|-----------------------------|----------|
| EC2 G6e.2xlarge  | 8 hrs/day × 22 days         | ~$352    |
| EBS gp3 100GB    | Always on                   | ~$8      |
| IoT Core         | ~15.8M msgs (10Hz × 8h × 22d) | ~$16  |
| Data Transfer    | ~10 GB out                  | ~$1      |
| CloudWatch       | Logs + metrics              | ~$3      |
| **Total**        |                             | **~$380** |

**Cost tips**: Stop instance outside demo hours (CDK output provides instance ID for easy stop/start). Spot instances possible but risk interruption during demos.

## Open Questions (Resolve After First AMI Boot)

1. What Isaac Sim version is on the AMI? Does it include the ROS2 bridge extension?
2. Is ROS2 pre-installed on the AMI, or does it need to be installed via UserData?
3. What are the exact joint prim names in the `SO-ARM101-USD.usd` action graph?
4. Does the USD action graph subscribe to `/joint_states` or a different topic?
5. Does Isaac Sim's Python use system Python or its own bundled Python?
