#!/bin/bash
set -euxo pipefail
exec > /var/log/dt-bootstrap.log 2>&1

echo "=== Digital Twin Bootstrap Starting ==="
date

# --- System updates ---
apt-get update -y
apt-get install -y curl gnupg2 lsb-release software-properties-common python3-pip

# --- ROS2 Jazzy (if not pre-installed) ---
if ! command -v ros2 &>/dev/null; then
    echo "Installing ROS2 Jazzy..."
    curl -sSL --proto =https https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list
    apt-get update -y
    apt-get install -y ros-jazzy-ros-base ros-jazzy-sensor-msgs ros-jazzy-std-msgs
    # NOTE: Do NOT add ROS2 to the login profile (/etc/profile.d).
    # It conflicts with Isaac Sim's internal ROS2 libs (Python 3.12 vs 3.11).
    # The bridge service sources ROS2 independently in its ExecStart.
else
    echo "ROS2 already installed: $(ros2 --version)"
fi

# --- Python dependencies ---
pip3 install --break-system-packages awsiotsdk boto3 || pip3 install awsiotsdk boto3

# Also install into Isaac Sim's Python if available
ISAAC_PYTHON=$(find /opt -name "python.sh" -path "*/isaac*" 2>/dev/null | head -1)
if [ -n "$ISAAC_PYTHON" ]; then
    echo "Installing deps into Isaac Sim Python..."
    $ISAAC_PYTHON -m pip install awsiotsdk boto3 || true
fi

# --- Create application directories ---
APP_DIR=/opt/digital-twin
mkdir -p $APP_DIR/assets $APP_DIR/logs

# Set ownership for the service user
DEFAULT_USER=$(getent passwd 1000 | cut -d: -f1 || echo "ubuntu")
chown -R $DEFAULT_USER:$DEFAULT_USER $APP_DIR

# --- Download USD model ---
if [ ! -f "$APP_DIR/assets/SO-ARM101-USD.usd" ]; then
    echo "Downloading USD model..."
    curl -sSL --proto =https -o $APP_DIR/assets/SO-ARM101-USD.usd \
        "https://raw.githubusercontent.com/MuammerBay/so-arm101-ros2-bridge/main/IsaacSim_USD/SO-ARM101-USD.usd" || {
        echo "WARNING: Failed to download USD model. Will need manual download."
    }
fi

# Note: Backend code is deployed via CDK S3 assets in UserData before this script runs

# --- NICE DCV session ---
if command -v dcv &>/dev/null; then
    echo "Setting up NICE DCV session..."
    DEFAULT_USER=$(getent passwd 1000 | cut -d: -f1 || echo "ubuntu")

    # Restart DCV server so it picks up the IMDS license before session creation
    systemctl restart dcvserver
    sleep 5

    dcv create-session --type=console --owner "$DEFAULT_USER" dt-session 2>/dev/null || {
        echo "DCV session may already exist or DCV not fully configured"
    }
fi

# --- DCV password: generate and store in Secrets Manager ---
echo "Setting up DCV password..."
DCV_PASSWORD=$(openssl rand -base64 16 | tr -d '/+=' | head -c 20)
DEFAULT_USER=$(getent passwd 1000 | cut -d: -f1 || echo "ubuntu")
echo "$DEFAULT_USER:$DCV_PASSWORD" | chpasswd

# Store in Secrets Manager (create or update)
# IMDSv2 requires a token for metadata access
IMDS_TOKEN=$(curl -s -X PUT "http://169.254.169.254/latest/api/token" \
    -H "X-aws-ec2-metadata-token-ttl-seconds: 60" --max-time 5 || true)
REGION=$(curl -s -H "X-aws-ec2-metadata-token: $IMDS_TOKEN" \
    --max-time 5 http://169.254.169.254/latest/meta-data/placement/region || true)
if [ -z "$REGION" ]; then
    REGION="us-east-1"
fi
aws secretsmanager create-secret \
    --name "isaac-sim-dt/dcv-password" \
    --secret-string "$DCV_PASSWORD" \
    --region "$REGION" 2>/dev/null || \
aws secretsmanager update-secret \
    --secret-id "isaac-sim-dt/dcv-password" \
    --secret-string "$DCV_PASSWORD" \
    --region "$REGION"
echo "DCV password stored in Secrets Manager: isaac-sim-dt/dcv-password"

# --- Systemd service for the bridge ---
# Detect user before creating service file
DEFAULT_USER=$(getent passwd 1000 | cut -d: -f1 || echo "ubuntu")

cat > /etc/systemd/system/dt-bridge.service << UNIT
[Unit]
Description=Digital Twin MQTT-ROS2 Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$DEFAULT_USER
WorkingDirectory=/opt/digital-twin/backend
ExecStartPre=/bin/test -f /opt/digital-twin/backend/bridge.py
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash 2>/dev/null; exec python3 /opt/digital-twin/backend/bridge.py'
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1
Environment=PYTHONPATH=/opt/digital-twin/backend

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload

# Enable service only if bridge.py has been deployed
if [ -f /opt/digital-twin/backend/bridge.py ]; then
    systemctl enable dt-bridge.service
    echo "dt-bridge service enabled"
else
    echo "WARNING: bridge.py not found — dt-bridge service not enabled"
    echo "Deploy backend scripts, then run: sudo systemctl enable --now dt-bridge"
fi

# Final ownership check
chown -R $DEFAULT_USER:$DEFAULT_USER $APP_DIR

echo "=== Digital Twin Bootstrap Complete ==="
date
