#!/bin/bash
# 割草机控制面板 —— 环境配置
# 用法: bash setup.sh
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 割草机 控制面板环境配置 ==="
echo "目录: $HERE"

# 1. 安装依赖
echo "[1/4] 检查依赖..."
if ! python3 -c "import serial; import cv2; import pyqtgraph; from PyQt5.QtWidgets import QWidget" 2>/dev/null; then
    echo "安装依赖包..."
    sudo apt install python3-pyqt5 -y
    pip3 install pyserial opencv-contrib-python-headless pyqtgraph numpy rplidar Pillow --break-system-packages -i https://pypi.tuna.tsinghua.edu.cn/simple
fi

# 2. 配置 udev 设备别名
# 别名按 USB 芯片 ID 认，不看插在哪个口 —— 换口、重插都还是这个名字。
echo "[2/4] 配置硬件规则..."
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="mower_chassis"' | sudo tee /etc/udev/rules.d/98-mower-chassis.rules > /dev/null
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="mower_lidar"' | sudo tee /etc/udev/rules.d/97-mower-lidar.rules > /dev/null
sudo udevadm trigger

# 3. 串口检测
echo "[3/4] 检测设备..."
[ -e /dev/mower_chassis ] && echo "底盘: /dev/mower_chassis ✔" || echo "底盘: 未检测到 —— 桥会起不来"
[ -e /dev/mower_lidar ] && echo "雷达: /dev/mower_lidar ✔" || echo "雷达: 未检测到"

# 4. 桌面图标
# 原来写死 /home/$USER/robot_ui，目录实际叫 mower-host —— 那个图标一直是坏的。
echo "[4/4] 桌面图标..."
cat > ~/Desktop/割草机控制面板.sh << DESK
#!/bin/bash
export DISPLAY=:0
cd "$HERE"
python3 main.py
DESK
chmod +x ~/Desktop/割草机控制面板.sh

echo "=== 配置完成 ==="
echo
echo "底盘桥要单独 colcon build 一次（之后只有改桥的代码才需要重来）："
echo "  source /opt/ros/humble/setup.bash"
echo "  cd $HERE/bridge && colcon build"
