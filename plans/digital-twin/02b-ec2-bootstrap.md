# Phase 2b: EC2 Bootstrap Script (UserData)

## Goal

Cloud-init / UserData script that configures the Isaac Sim EC2 instance on first boot: install ROS2 Jazzy, Python dependencies, download the USD model, and set up systemd services.

## Dependencies

- Phase 1 (EC2 instance must exist)
- Can run in parallel with Phase 2 (subscriber code development)

## Tasks

### 2b.1 Environment Discovery Script

First boot should verify what's already on the AMI before installing anything:

```bash
#!/bin/bash
# discovery.sh — run manually after first launch to verify AMI contents

echo "=== GPU ==="
nvidia-smi

echo "=== Isaac Sim ==="
ls /opt/isaac-sim* 2>/dev/null || ls ~/.local/share/ov/pkg/isaac* 2>/dev/null
cat /opt/isaac-sim*/VERSION 2>/dev/null

echo "=== Isaac Sim Python ==="
/opt/isaac-sim*/python.sh --version 2>/dev/null

echo "=== ROS2 ==="
which ros2 && ros2 --version
ls /opt/ros/ 2>/dev/null

echo "=== NICE DCV ==="
which dcv && dcv version
dcv list-sessions 2>/dev/null

echo "=== ROS2 Bridge Extension ==="
ls /opt/isaac-sim*/exts/ 2>/dev/null | grep -i ros

echo "=== Python ==="
python3 --version
pip3 --version
```

**Important**: Run this manually before finalizing the UserData script. The AMI may already include ROS2, DCV, and other deps.

### 2b.2 UserData Script (Template)

Adjust based on discovery results:

```bash
#!/bin/bash
set -euxo pipefail
exec > /var/log/userdata.log 2>&1

# --- System updates ---
apt-get update -y

# --- ROS2 Jazzy (if not pre-installed) ---
# Check if ROS2 is already available
if ! command -v ros2 &>/dev/null; then
  apt-get install -y software-properties-common
  add-apt-repository universe
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -y
  apt-get install -y ros-jazzy-ros-base ros-jazzy-sensor-msgs
fi

# --- Python dependencies ---
pip3 install awsiotsdk boto3

# Also install into Isaac Sim's Python if it has its own environment
ISAAC_PYTHON=$(find /opt/isaac-sim* -name "python.sh" 2>/dev/null | head -1)
if [ -n "$ISAAC_PYTHON" ]; then
  $ISAAC_PYTHON -m pip install awsiotsdk boto3
fi

# --- Download USD model ---
ASSET_DIR=/opt/digital-twin/assets
mkdir -p $ASSET_DIR
curl -sSL -o $ASSET_DIR/SO-ARM101-USD.usd \
  "https://raw.githubusercontent.com/MuammerBay/so-arm101-ros2-bridge/main/IsaacSim_USD/SO-ARM101-USD.usd"

# --- Deploy backend scripts ---
BACKEND_DIR=/opt/digital-twin/backend
mkdir -p $BACKEND_DIR
# Scripts will be delivered via CDK S3 asset or inline in UserData
# For now, copy from /home/ubuntu/backend if deployed via CDK asset

# --- NICE DCV setup (if not pre-configured) ---
if command -v dcv &>/dev/null; then
  # Create a console session for the default user
  dcv create-session --type=console --owner ubuntu dt-session 2>/dev/null || true
fi

# --- Signal completion ---
echo "UserData bootstrap complete" >> /var/log/userdata.log
```

### 2b.3 Systemd Service for MQTT Bridge

```ini
# /etc/systemd/system/dt-bridge.service
[Unit]
Description=Digital Twin MQTT-ROS2 Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/opt/digital-twin/backend
ExecStart=/usr/bin/python3 /opt/digital-twin/backend/bridge.py
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
```

### 2b.4 CDK UserData Integration

```typescript
const userData = ec2.UserData.forLinux();
userData.addCommands(
  // Read and inline the bootstrap script, or use S3 asset
  'set -euxo pipefail',
  'exec > /var/log/userdata.log 2>&1',
  // ... bootstrap commands ...
);
```

Alternative: Upload backend scripts as an S3 asset via CDK and download in UserData.

## Acceptance Criteria

- [ ] Discovery script runs and documents AMI contents
- [ ] ROS2 Jazzy installed (or confirmed pre-installed)
- [ ] `awsiotsdk` and `boto3` installed in system Python
- [ ] USD model downloaded to `/opt/digital-twin/assets/`
- [ ] Backend scripts deployed to `/opt/digital-twin/backend/`
- [ ] NICE DCV session created automatically
- [ ] Systemd service configured for the bridge process
