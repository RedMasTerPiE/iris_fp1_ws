"""Mode 2: drive to a point you pick, with Ackermann steering.

Starts the whole control stack:

    udpSender     keyboard + /drive_command  ->  UDP to the car
    udpReceiver   UDP telemetry from the car  ->  /car/telemetry
    poseEstimator /car/telemetry              ->  /car/pose
    ackermannSteering  /ackermann/target + /car/pose -> /drive_command

    ros2 launch master ackermann.launch.py
    ros2 launch master ackermann.launch.py target_x:=10.0 target_y:=0.0

The target is in the map frame: x east, y north, with the car starting at
the origin facing +x. It only has to be sent once: the node keeps the
command alive and refreshes it at 20 Hz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    publish_target = LaunchConfiguration("publish_target")
    car_x = LaunchConfiguration("car_x")
    car_y = LaunchConfiguration("car_y")
    car_theta = LaunchConfiguration("car_theta_deg")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="address of the car or of the simulator"),
        DeclareLaunchArgument("port", default_value="12151",
                              description="UDP port the commands go to"),
        DeclareLaunchArgument("max_pwm", default_value="150"),
        DeclareLaunchArgument("max_steer", default_value="25"),
        DeclareLaunchArgument("steer_wire_sign", default_value="-1",
                              description="sign of the steering byte on the wire: "
                                          "-1 for the simulator, +1 if the car "
                                          "wants a positive angle to mean left"),
        DeclareLaunchArgument("telemetry_port", default_value="12152",
                              description="UDP port the car's telemetry arrives on"),
        DeclareLaunchArgument("target_x", default_value="0.0",
                              description="x of the point to drive to (map frame)"),
        DeclareLaunchArgument("target_y", default_value="0.0",
                              description="y of the point to drive to (map frame)"),
        DeclareLaunchArgument("publish_target", default_value="true",
                              description="send the target once at startup"),
        DeclareLaunchArgument("car_x", default_value="0.0",
                              description="where the car is, if no pose has arrived"),
        DeclareLaunchArgument("car_y", default_value="0.0"),
        DeclareLaunchArgument("car_theta_deg", default_value="0.0",
                              description="which way the car faces, in degrees"),

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
            parameters=[{
                "car_x": ParameterValue(car_x, value_type=float),
                "car_y": ParameterValue(car_y, value_type=float),
                "car_theta_deg": ParameterValue(car_theta, value_type=float),
            }],
        ),

        # the target only has to be sent once, the node keeps it
        ExecuteProcess(
            cmd=["ros2", "topic", "pub", "--once", "/ackermann/target",
                 "geometry_msgs/msg/Point",
                 "{x: " + target_x + ", y: " + target_y + ", z: 0.0}"],
            output="screen",
            condition=IfCondition(publish_target),
        ),
    ])
