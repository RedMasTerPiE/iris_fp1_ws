"""Mode 3: fully autonomous.

Same control stack as mode 2, but nobody publishes a target: the car is
supposed to pick its own goals from what it sees, and to keep driving.

    ros2 launch master autonomous.launch.py

What is still missing: the vision package has no node yet, so nothing
publishes /ackermann/target on its own. Until that node exists, this
launch file behaves like mode 2 without a target, which means the car
stands still. The place to add the vision node is marked below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    max_pwm = LaunchConfiguration("max_pwm")
    max_steer = LaunchConfiguration("max_steer")
    wire_sign = LaunchConfiguration("steer_wire_sign")
    telemetry_port = LaunchConfiguration("telemetry_port")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="12151"),
        DeclareLaunchArgument("max_pwm", default_value="150"),
        DeclareLaunchArgument("max_steer", default_value="25"),
        DeclareLaunchArgument("steer_wire_sign", default_value="-1",
                              description="sign of the steering byte on the wire: "
                                          "-1 for the simulator, +1 if the car "
                                          "wants a positive angle to mean left"),
        DeclareLaunchArgument("telemetry_port", default_value="12152"),

        Node(
            package="communication",
            executable="udpSender",
            name="udp_sender",
            output="screen",
            arguments=[host, port, max_pwm, max_steer],
            parameters=[{"steer_wire_sign": ParameterValue(wire_sign, value_type=int)}],
        ),
        Node(
            package="communication",
            executable="udpReceiver",
            name="udp_receiver",
            output="screen",
            arguments=[telemetry_port],
        ),
        Node(
            package="master",
            executable="poseEstimator",
            name="pose_estimator",
            output="screen",
        ),
        Node(
            package="master",
            executable="ackermannSteering",
            name="ackermann_steering",
            output="screen",
        ),

        # The vision node goes here: it reads the camera and publishes the
        # point the car should drive to on /ackermann/target, for example
        #
        # Node(
        #     package="vision",
        #     executable="laneFollower",
        #     name="lane_follower",
        #     output="screen",
        # ),
    ])
