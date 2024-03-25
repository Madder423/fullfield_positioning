import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    robot_description_pkg = "robot_description"
    robot_description_pkg_path = get_package_share_directory(robot_description_pkg)

    TF_board_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(robot_description_pkg_path, f'launch/TF_boardcast.launch.py')
    )
    )
    ld.add_action(TF_board_launch)
    odom_node = Node(
        package="fullfield_positioning",
        executable="odometer_publisher"
    )
    ld.add_action(odom_node)
    # 返回让ROS2根据launch描述执行节点
    return ld
