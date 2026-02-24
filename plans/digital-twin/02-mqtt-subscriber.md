# Phase 2: IoT Core MQTT Subscriber

## Goal

Python module that subscribes to IoT Core via MQTT5/WebSocket (SigV4), receives 10Hz telemetry, converts ticks to radians, and exposes the latest joint state via a thread-safe buffer.

## Dependencies

- Phase 1 (IAM role must grant IoT permissions)

## File Structure

```
/backend
├── requirements.txt
├── subscriber.py          # MQTT5 subscriber + telemetry buffer
├── joint_converter.py     # Ticks → radians conversion
└── config_loader.py       # Reads config.json
```

## Tasks

### 2.1 Config Loader

Read `config.json` from project root. Extract:
- `iot.topicPrefix` + `iot.deviceId` → full topic `dt/lerobot/arm-001/telemetry`
- `aws.region` → SigV4 signing region

### 2.2 Joint Converter

```python
import math

TICKS_PER_REV = 4096
JOINT_NAMES = [
    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
    'wrist_flex', 'wrist_roll', 'gripper'
]

def ticks_to_radians(ticks: int) -> float:
    return ticks * (2 * math.pi / TICKS_PER_REV)

def convert_payload(payload: dict) -> dict:
    """Convert IoT payload to radians dict.

    Input:  {"joints": {"shoulder_pan": {"position": 2048, ...}, ...}}
    Output: {"shoulder_pan": 3.14159, "shoulder_lift": ..., ...}
    """
    joints = payload.get('joints', {})
    return {
        name: ticks_to_radians(joints[name]['position'])
        for name in JOINT_NAMES
        if name in joints
    }
```

### 2.3 Telemetry Buffer

Thread-safe single-slot buffer — MQTT callback overwrites, consumer reads latest.

```python
import threading

class TelemetryBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
        self._timestamp = 0

    def update(self, payload: dict):
        with self._lock:
            self._data = payload
            self._timestamp = payload.get('timestamp', 0)

    def get_latest(self) -> dict | None:
        with self._lock:
            return self._data

    @property
    def last_timestamp(self) -> int:
        with self._lock:
            return self._timestamp
```

### 2.4 MQTT5 Subscriber

Key implementation details:

1. **Endpoint discovery**: `boto3.client('iot').describe_endpoint(endpointType='iot:Data-ATS')`
2. **Client builder**: `mqtt5_client_builder.websockets_with_default_aws_signing()`
3. **Credentials**: Default chain picks up EC2 instance profile via IMDS
4. **Client ID**: `isaac-sim-dt-{timestamp}` to avoid conflicts on restart
5. **Subscribe in `on_lifecycle_connection_success`** — ensures re-subscribe after reconnect
6. **QoS 0** — matches publisher, lowest latency

### 2.5 requirements.txt

```
awsiotsdk>=1.28.0
boto3>=1.34.0
```

Note: `awsiotsdk` pulls in `awscrt` (compiled C extension for high-perf TLS/event loop).

## Acceptance Criteria

- [ ] Subscriber connects to IoT Core using EC2 instance profile credentials
- [ ] Receives messages on `dt/lerobot/arm-001/telemetry`
- [ ] Converts position ticks to radians correctly (2048 ticks = π)
- [ ] TelemetryBuffer correctly overwrites stale data
- [ ] Auto-reconnects on connection drop
- [ ] Logs connection events and message rate
