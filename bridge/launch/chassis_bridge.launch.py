from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower_bridge',
            executable='chassis_bridge',
            name='chassis_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/mower_chassis',
                'baud': 115200,
                # 停发 /cmd_vel 多久算丢指令。固件那道保护够不着（见 node.py 顶部
                # 说明），停车只能靠这个。
                'cmd_vel_timeout': 0.5,

                # 这三个必须和我们自己的 bringup/ekf.yaml 对齐，否则 ekf 不出 TF：
                #   odom_frame: odom_combined / base_link_frame: base_footprint
                #   imu0: /imu/data_raw
                'odom_frame_id': 'odom_combined',
                'robot_frame_id': 'base_footprint',
                'gyro_frame_id': 'gyro_link',

                # 里程计标定旋钮。默认全 1.0 = 没标定过，是留给实测的。
                # 走 1 m 差多少，按比例改这里（前进/后退各一个）。
                'odom_x_scale': 1.0,
                'odom_y_scale': 1.0,
                'odom_z_scale_positive': 1.0,
                'odom_z_scale_negative': 1.0,
            }],
        ),
    ])
