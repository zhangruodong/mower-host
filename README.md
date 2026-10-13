# 割草机上位机（mower-host）

树莓派上跑的控制面板，一块 PyQt5 界面加一个 ROS2 底盘串口桥。

上机前先读 [DISCLAIMER.md](DISCLAIMER.md)。

## 功能

- PyQt5 控制面板，界面和按钮都写在 `ui/main_window.py`
- ROS2 底盘串口桥，`bridge/` 里的 `chassis_bridge` 节点把 `/cmd_vel` 与 `/dev/mower_chassis` 互相转换，装上就能控底盘
- 回充舵机扫描
- 11 字节帧协议，备用路径，可以脱机自检

## 硬件与依赖

底盘是 STM32F407 加一块驱动板，雷达用 RPLidar C1，USB 转串口那颗芯片是 CH340，设备 ID `1a86:55d4`。

软件要装 ROS2 Humble，另外装 `robot_localization`，`/odom_combined` 和 TF 由它出。要建图导航再装 `slam_toolbox` 或 Nav2。

## 目录结构

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

## 安装

```bash
bash setup.sh                                   # 依赖 + udev 别名 + 桌面图标
source /opt/ros/humble/setup.bash
cd bridge && colcon build                       # 桥要单独编一次
```

`setup.sh` 装 pip 依赖、写 udev 规则、放一个桌面启动图标，不编 ROS 包。udev 规则让串口固定叫 `/dev/mower_chassis`，雷达固定叫 `/dev/mower_lidar`。

## 运行

```bash
source /opt/ros/humble/setup.bash
source bridge/install/setup.bash
ros2 launch mower_bridge chassis_bridge.launch.py     # 终端 1
python3 main.py                                        # 终端 2
```

树莓派上的绝对路径有三处，写在 [ui/main_window.py](ui/main_window.py) 开头：`ROS_WS` 是 SLAM/Nav2 的工作空间，`MAP_DIR` 是地图存放目录，`RVIZ_CFG` 指向 rviz 配置。换机器改这三个常量。

界面上「启动系统」按钮会自动带上上面两条 source，定义在 `main_window.py` 的 `BRIDGE_SETUP` 和 `BRIDGE_LAUNCH`，路径按文件位置推，换目录不用改。

## 脱机自检

没接底盘、甚至在 Windows 上也能开界面。`ros2_client.py` 在 ROS 没起来的时候静默丢弃发送、不发状态，界面本身不崩。协议层可以脱机跑：

```bash
python3 core/protocol.py        # 组帧字节序列自检
python3 bridge/test/test_protocol.py   # 11 项：组帧 / 解析 / 重同步 / Mahony
```

接底盘之前，可以拿一对虚拟串口把桥整个跑通，先 `apt install socat`：

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0     # 记下它打印的两个 /dev/pts/N
ros2 launch mower_bridge chassis_bridge.launch.py port:=/dev/pts/3
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
cat /dev/pts/4 | xxd                           # 应看到 7B 00 00 01 F4 00 00 00 00 8E 7D 重复
ros2 topic echo /odom                          # 往 /dev/pts/4 灌帧，看有没有输出
```

停掉 `ros2 topic pub` 之后 **0.5 秒内**应看到一帧全零速度，这是看门狗在起作用。

## 文档

- [docs/功能.md](docs/功能.md)：每颗按钮管什么、桥收发哪些话题、文件落在哪，末尾一节写着还没做的部分断在哪一行。
- [docs/协议.md](docs/协议.md)：11/24 字节帧完整规格、每个 CMD、上电静默期。改桥之前先看这份。
- [docs/架构.md](docs/架构.md)：上下位机怎么分、看门狗为什么必须由桥来兜。
- [docs/移除记录.md](docs/移除记录.md)：删掉的东西和删的理由。

## 注意事项

- 串口同一时刻只能有一个进程开，`chassis_bridge` 起来之后别的底盘节点就不能再开，后开的那个抢不到 `/dev/mower_chassis`。
- 雷达装在哪个位置、朝哪个角度，`ekf.yaml` 里的系数，还有串口设备名，都是照这台车量的。换车或者把雷达挪个地方，这些都得重量一遍。
- [DISCLAIMER.md](DISCLAIMER.md) 上机前必读：这台车什么情况下停不下来、刀片归谁管、为什么必须另装一个物理急停。
- 刀片控制出于安全已整套移除，上位机不再提供任何刀片控制路径。删了什么见 [docs/移除记录.md](docs/移除记录.md)。

## 许可

[LICENSE](LICENSE)：MIT。

上位机和下位机是两个仓库，这份只有上位机。下位机的 STM32 固件还没整理完，暂不发布。
