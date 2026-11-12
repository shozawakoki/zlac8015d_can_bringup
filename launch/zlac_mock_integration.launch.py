"""CAN-free integration bringup for diff_drive_controller and mocked wheel hardware.

THIS LAUNCH DOES NOT LOAD zlac8015d_hardware OR ACCESS CAN.
It replaces the hardware plugin in a private, in-memory URDF.
Do not use alongside an existing controller_manager or a running robot.
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def build_mock_description(urdf_path):
    tree = ET.parse(urdf_path)
    hardware = tree.find(".//ros2_control/hardware")
    if hardware is None:
        raise RuntimeError("Missing ros2_control hardware")
    plugins = hardware.findall("plugin")
    if len(plugins) != 1:
        raise RuntimeError("Expected one hardware plugin")
    plugins[0].text = "mock_components/GenericSystem"
    # CAN/driver-only parameters are forbidden even in the mock description.
    for child in list(hardware):
        if child.tag == "param":
            hardware.remove(child)
    result = ET.tostring(tree.getroot(), encoding="unicode")
    if "zlac8015d_hardware/Zlac8015dSystemHardware" in result:
        raise RuntimeError("Real CAN hardware plugin remained in mock URDF")
    return result


def _nodes(context):
    del context
    share = get_package_share_directory("zlac_can_bringup")
    description = build_mock_description(
        os.path.join(share, "urdf", "orange_go_2025.urdf"))
    controller_path = os.path.join(share, "config", "controllers.yaml")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": description}, controller_path],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
                ("/diff_drive_controller/odom", "/odom"),
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_nodes)])
