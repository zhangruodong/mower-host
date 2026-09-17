# 割草机上位机（mower-host）

树莓派上跑的控制面板：一块 PyQt5 界面 + 一个 ROS2 底盘串口桥。

本工程是针对**我们这一台割草机**的特定硬件（串口设备名、雷达安装位置与角度、`ekf.yaml`
的标定系数等均按实车调）写的参考实现，不是通用产品软件——换一台车、换一套接线、把雷达
挪个位置，相关参数都要重新标定。它控制的是真实的机电设备，**上机前先读
[DISCLAIMER.md](DISCLAIMER.md)**。

```
┌──────────────────────────────────────────────────────┐
│  main.py            PyQt5 主窗口                     │
├──────────────────────────────────────────────────────┤
│  core/ros2_client.py   话题                          │
│  core/protocol.py      11 字节帧（备用/离线自检）    │
│  core/dock_controller.py  回充舵机扫描               │
│  ui/main_window.py     所有界面和按钮                │
├──────────────────────────────────────────────────────┤
│  bridge/  ← ROS2 包 mower_bridge                     │
│    chassis_bridge 节点：/cmd_vel ↔ /dev/mower_chassis│
└──────────────────────────────────────────────────────┘
```

## 需要什么

**硬件**：底盘（STM32F407 + 驱动板）、RPLidar C1、USB 转串口（CH340，`1a86:55d4`）。

串口收发由 `bridge/` 里的 `chassis_bridge` 包了——装上它就能控制底盘。

需要装的是 ROS2 Humble + `robot_localization`（出 `/odom_combined` 和 TF）+ `slam_toolbox` / Nav2（要建图导航才装）。

## 装

```bash
bash setup.sh                                   # 依赖 + udev 别名 + 桌面图标
source /opt/ros/humble/setup.bash
cd bridge && colcon build                       # 桥要单独编一次
```

`setup.sh` 只做三件事：装 pip 依赖、写 udev 规则让串口固定叫 `/dev/mower_chassis`（雷达是 `/dev/mower_lidar`）、在桌面放个启动图标。它**不**编 ROS 包——ROS 环境得自己 source。

## 跑

```bash
source /opt/ros/humble/setup.bash
source bridge/install/setup.bash
ros2 launch mower_bridge chassis_bridge.launch.py     # 终端 1
python3 main.py                                        # 终端 2
```

界面上「启动系统」那个按钮会自动带上这两条 source（`main_window.py` 里的 `BRIDGE_SETUP` / `BRIDGE_LAUNCH`），路径按文件位置推，换目录不用改。

树莓派上的绝对路径集中写在 [ui/main_window.py](ui/main_window.py) 开头三个常量里：`ROS_WS`（SLAM/Nav2 那个工作空间）、`MAP_DIR`（地图存哪）、`RVIZ_CFG`（rviz 配置）。换机器只改这三行。

**串口同一时刻只能有一个进程开**——`chassis_bridge` 起来了，别的底盘节点就不能再开，后开的那个抢不到 `/dev/mower_chassis`。

## 没有硬件时

Windows / 没接底盘也能开界面：`ros2_client.py` 在 ROS 没起来时静默丢弃发送、不发状态，UI 本身不崩。协议层可以脱机自检：

```bash
python3 core/protocol.py        # 组帧字节序列自检
python3 bridge/test/test_protocol.py   # 11 项：组帧 / 解析 / 重同步 / Mahony
```

在树莓派上、接底盘之前，可以拿一对虚拟串口把桥整个跑通（`apt install socat`）：

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0     # 记下它打印的两个 /dev/pts/N
ros2 launch mower_bridge chassis_bridge.launch.py port:=/dev/pts/3
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
cat /dev/pts/4 | xxd                           # 应看到 7B 00 00 01 F4 00 00 00 00 8E 7D 重复
ros2 topic echo /odom                          # 往 /dev/pts/4 灌帧，看有没有输出
```

停掉 `ros2 topic pub` 后 **0.5 秒内**应看到一帧全零速度——那是看门狗，不是 bug。

## 文档

- [docs/功能.md](docs/功能.md) —— **这面板能干什么**：每颗按钮管什么、桥收发哪些话题、文件落在哪；末尾一节写着**还没做的断在哪一行**。
- [docs/协议.md](docs/协议.md) —— 11/24 字节帧完整规格、每个 CMD、上电静默期。**改桥之前先看这份。**
- [docs/架构.md](docs/架构.md) —— 上下位机怎么分的、为什么看门狗必须由桥来兜。
- [docs/移除记录.md](docs/移除记录.md) —— 删掉的东西留档 + 为什么删。

## 许可与安全

- [LICENSE](LICENSE) —— MIT。
- [DISCLAIMER.md](DISCLAIMER.md) —— **上机前必读**：这台车停不下来是什么情况、刀片归谁管、
  为什么必须另装物理急停。
- **刀片控制出于安全已整套移除**，上位机不再提供任何刀片控制路径。删了什么见
  [docs/移除记录.md](docs/移除记录.md)。

上位机和下位机是**两个仓库**，这份只有上位机。下位机固件（STM32）还没整理完，暂不发布。
