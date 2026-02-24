#!/bin/bash
# Run this after first launch to verify AMI contents
echo "=== GPU ==="
nvidia-smi 2>/dev/null || echo "nvidia-smi not found"

echo ""
echo "=== Isaac Sim ==="
for dir in /opt/isaac-sim* /opt/nvidia/isaac-sim* ~/.local/share/ov/pkg/isaac*; do
    [ -d "$dir" ] && echo "Found: $dir" && cat "$dir/VERSION" 2>/dev/null
done

echo ""
echo "=== Isaac Sim Python ==="
ISAAC_PYTHON=$(find /opt -name "python.sh" -path "*/isaac*" 2>/dev/null | head -1)
[ -n "$ISAAC_PYTHON" ] && $ISAAC_PYTHON --version 2>/dev/null || echo "Not found"

echo ""
echo "=== ROS2 ==="
which ros2 2>/dev/null && ros2 --version 2>/dev/null || echo "Not installed"
ls /opt/ros/ 2>/dev/null || echo "No /opt/ros"

echo ""
echo "=== NICE DCV ==="
which dcv 2>/dev/null && dcv version 2>/dev/null || echo "Not installed"
dcv list-sessions 2>/dev/null || true

echo ""
echo "=== ROS2 Bridge Extension ==="
find /opt -name "*.so" -path "*ros2*bridge*" 2>/dev/null | head -5 || echo "Not found"
ls /opt/isaac-sim*/exts/ 2>/dev/null | grep -i ros || true

echo ""
echo "=== System Python ==="
python3 --version
pip3 --version 2>/dev/null

echo ""
echo "=== Network ==="
curl -s --max-time 5 https://checkip.amazonaws.com || echo "No internet"
