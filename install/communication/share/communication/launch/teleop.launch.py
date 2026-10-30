"""Mode 1: WASD teleop.

Only the sender is needed: it reads the keyboard and streams the commands
over UDP to the car.

    ros2 launch communication teleop.launch.py
    ros2 launch communication teleop.launch.py host:=192.168.1.10 port:=12151

The keyboard is grabbed while this runs, so the keys do not reach the
terminal. Ctrl-C stops it and sends a final stop packet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    max_pwm = LaunchConfiguration("max_pwm")
    max_steer = LaunchConfiguration("max_steer")
    wire_sign = LaunchConfiguration("steer_wire_sign")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="address of the car or of the simulator"),
        DeclareLaunchArgument("port", default_value="12151",
                              description="UDP port the commands go to"),
        DeclareLaunchArgument("max_pwm", default_value="150",
                              description="top speed of the keyboard, in PWM units"),
        DeclareLaunchArgument("max_steer", default_value="25",
                              description="steering angle of the keyboard, in degrees"),
        DeclareLaunchArgument("steer_wire_sign", default_value="-1",
                              description="sign of the steering byte on the wire: "
                                          "-1 for the simulator, +1 if the car "
                                          "wants a positive angle to mean left"),
        Node(
            package="communication",
            executable="udpSender",
            name="udp_sender",
            output="screen",
            arguments=[host, port, max_pwm, max_steer],
            parameters=[{"steer_wire_sign": wire_sign}],
        ),
    ])
