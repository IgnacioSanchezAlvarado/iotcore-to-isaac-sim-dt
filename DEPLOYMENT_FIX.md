# Backend Deployment Fix

## Problem
The EC2 instance bootstrap was creating `/opt/digital-twin/backend/` but not actually deploying the Python backend code. The `bootstrap.sh` script had a placeholder for `BACKEND_S3_URI` but the CDK stack never set it, resulting in missing `bridge.py` and a failed systemd service.

## Solution
Use CDK's S3 Asset mechanism to automatically upload backend code to S3 and download it during instance initialization.

## Changes Made

### 1. `/infra/lib/isaac-sim-stack.ts`
- Added import for `aws-cdk-lib/aws-s3-assets`
- Created two S3 Assets:
  - `BackendAsset`: Uploads entire `/backend` directory (excluding `scripts/`, `__pycache__/`, `*.pyc`)
  - `ConfigAsset`: Uploads `config.json` to the instance
- Granted EC2 role read access to both assets
- Added UserData commands that run BEFORE bootstrap.sh:
  - Downloads backend.zip from S3
  - Extracts to `/opt/digital-twin/backend/`
  - Downloads config.json to `/opt/digital-twin/config.json`
  - Logs to `/var/log/dt-asset-download.log`

### 2. `/backend/scripts/bootstrap.sh`
- Removed `BACKEND_S3_URI` conditional block (no longer needed)
- Removed `mkdir -p $APP_DIR/backend` (UserData creates this)
- Added comment explaining backend is deployed via CDK assets

## How It Works

1. During `cdk synth/deploy`, CDK uploads `/backend` and `/config.json` to a CDK-managed S3 bucket
2. The EC2 instance role gets read permissions for these objects
3. When the instance boots, UserData runs first:
   - Downloads and extracts backend code
   - Downloads config.json
4. Then bootstrap.sh runs:
   - Installs ROS2, Python deps
   - Downloads USD model
   - Configures DCV
   - Creates systemd service for `dt-bridge`
5. The systemd service checks for `/opt/digital-twin/backend/bridge.py` before enabling
   - With this fix, `bridge.py` will be present
   - Service will enable and start automatically

## Testing

After deploying the updated stack:

```bash
# SSH into the instance
aws ssm start-session --target <instance-id>

# Check that backend code was deployed
ls -la /opt/digital-twin/backend/
# Should see: bridge.py, subscriber.py, ros2_publisher.py, etc.

# Check that config was deployed
cat /opt/digital-twin/config.json

# Check asset download logs
sudo cat /var/log/dt-asset-download.log

# Check bootstrap logs
sudo cat /var/log/dt-bootstrap.log

# Check service status
sudo systemctl status dt-bridge.service
```

## Files Modified
- `/infra/lib/isaac-sim-stack.ts`
- `/backend/scripts/bootstrap.sh`
