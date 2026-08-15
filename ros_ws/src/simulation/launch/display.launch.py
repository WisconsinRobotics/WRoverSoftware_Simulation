import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro



def generate_launch_description():
    pkg_share = get_package_share_directory('test_rover')
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'test_rover','test_rover.urdf.xacro')

    # So right here what is like the point of doing this?
    urdf_arg = DeclareLaunchArgument(
       name='urdf_path',
        default_value=default_urdf_path,
        description='Absolute path to the URDF file'
    )
    # Process the Xacro file
    doc = xacro.process_file(default_urdf_path)
    robot_description_content = doc.toprettyxml(indent = "   ")


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
