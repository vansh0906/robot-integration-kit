"""Base driver for one partner platform.

    ros2 launch rik_ros base.launch.py partner:=partner_a_warehouse_amr
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg_dir = os.path.join(get_package_share_directory("rik_ros"), "configs")
    cfg = PythonExpression(["'", cfg_dir, "/' + '", LaunchConfiguration("partner"), "' + '.yaml'"])
    return LaunchDescription([
        DeclareLaunchArgument("partner", default_value="partner_a_warehouse_amr"),
        DeclareLaunchArgument("simulate", default_value="true"),
        Node(package="rik_ros", executable="base_driver", name="rik_base_driver", output="screen",
             parameters=[{"config_file": cfg, "simulate": LaunchConfiguration("simulate")}]),
    ])
