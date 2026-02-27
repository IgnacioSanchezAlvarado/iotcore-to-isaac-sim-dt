# Troubleshooting

## Bootstrap & Instance Startup

### DCV password secret not found

**Symptom**: `ResourceNotFoundException` when running:
```bash
aws secretsmanager get-secret-value --secret-id isaac-sim-dt/dcv-password
```

**Cause**: The EC2 instance bootstrap takes 3-5 minutes after deployment. The DCV password is stored in Secrets Manager near the end of the bootstrap process.

**Fix**: Wait for the bootstrap to finish. You can monitor progress via SSM:
```bash
# Find the instance ID
aws ec2 describe-instances \
  --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running \
  --query 'Reservations[].Instances[].InstanceId' --output text

# Connect and tail the bootstrap log
aws ssm start-session --target <instance-id>
sudo tail -f /var/log/dt-bootstrap.log
```

Look for `=== Digital Twin Bootstrap Complete ===` to confirm it finished. If the log ends with an error, see the sections below.

### Bootstrap failed — check both log files

The instance startup runs two phases, each with its own log:

| Phase | Log file | What it does |
|-------|----------|-------------|
| Asset download | `/var/log/dt-asset-download.log` | Downloads backend code and config from S3 |
| Bootstrap | `/var/log/dt-bootstrap.log` | Installs ROS2, Python deps, USD model, DCV setup |

If the bootstrap log is empty or missing, the asset download phase likely failed first.

### Instance not reachable via SSM

- The instance takes 1-2 minutes to register with SSM after launch.
- Verify the instance is running: `aws ec2 describe-instances --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running`
- If no instance is found, the ASG may have failed to launch (check EC2 console for capacity errors — G6e instances have limited availability).

## DCV Connection

### Can't connect to DCV (browser timeout)

1. **Check your IP**: The security group only allows access from the IP in `config.json` → `ec2.clientIpCidr`. If your IP changed, update the config and redeploy.
2. **Check the port**: DCV runs on port 8443 by default. Make sure you're using `https://<public-ip>:8443`.
3. **Check DCV is running** (via SSM):
   ```bash
   sudo systemctl status dcvserver
   dcv list-sessions
   ```
4. **Instance might not have a public IP** — verify:
   ```bash
   aws ec2 describe-instances \
     --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running \
     --query 'Reservations[].Instances[].PublicIpAddress' --output text
   ```

### DCV session not found

If `dcv list-sessions` returns empty, recreate it:
```bash
sudo dcv create-session --type=console --owner ubuntu dt-session
```

## Bridge Service (dt-bridge)

### Check bridge status

```bash
sudo systemctl status dt-bridge
sudo journalctl -u dt-bridge -f
```

### Bridge fails to start — ROS2 not found

If the logs show `source: not found` or `ros2: command not found`:
```bash
# Verify ROS2 is installed
ls /opt/ros/jazzy/setup.bash

# If missing, re-run the ROS2 install from bootstrap
sudo apt-get update -y
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-sensor-msgs ros-jazzy-std-msgs
```

### Bridge connects but no messages received

1. **Verify IoT Core topic** — the bridge subscribes to `dt/lerobot/arm-001/telemetry`. Confirm the edge device is publishing to this exact topic.
2. **Test with mock data** from your local machine:
   ```bash
   pip install boto3
   python tools/mock_publisher.py --rate 10
   ```
3. **Check IAM permissions** — the instance role needs `iot:Connect`, `iot:Subscribe`, and `iot:Receive`. Verify via:
   ```bash
   aws sts get-caller-identity   # confirms the instance has credentials
   aws iot describe-endpoint --endpoint-type iot:Data-ATS   # confirms IoT access
   ```

### Bridge running but Isaac Sim not moving

- Confirm Isaac Sim is open with the USD scene loaded (`/opt/digital-twin/assets/SO-ARM101-USD.usd`).
- Check that the ROS2 action graph in the USD scene is active. In Isaac Sim, open the Action Graph panel and verify the `/joint_states` subscriber node is connected.
- Verify ROS2 messages are flowing:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /joint_states
  ```

## GPU & Isaac Sim

### Isaac Sim won't start — GPU not detected

```bash
nvidia-smi   # should show the L40S GPU
```

If `nvidia-smi` fails, the NVIDIA drivers may not have loaded. Reboot the instance:
```bash
sudo reboot
```

The Isaac Sim AMI ships with pre-installed drivers, but they occasionally need a reboot after first launch.

## Redeploying After Fixes

After updating code (bootstrap script, backend, etc.), redeploy and cycle the instance:

```bash
cd infra
npx cdk deploy IsaacSimDTStack
```

The ASG will continue running the old instance. To pick up new UserData, terminate the current instance and let the ASG launch a fresh one:

```bash
# Find and terminate the running instance
INSTANCE_ID=$(aws ec2 describe-instances \
  --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running \
  --query 'Reservations[].Instances[].InstanceId' --output text)

aws ec2 terminate-instances --instance-ids $INSTANCE_ID
```

The ASG will automatically launch a new instance with the updated bootstrap.
