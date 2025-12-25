"""Launch file to display the simple rover in RViz using robot_state_publisher"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('nxt_rover')
    xacro_path = os.path.join(pkg_share, 'urdf', 'simple_rover.urdf.xacro')

    # Process xacro to produce an URDF string
    robot_description = {'robot_description': xacro.process_file(xacro_path).toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([rsp_node])
