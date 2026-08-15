# © Siemens AG, 2024
# Author: Mehmet Emre Cakal (emre.cakal@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ---
# Last Editor: Christopher Teggatz
# This is responsible for publishing the test_rover

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():


    ####  Define launch arguments ####
    port_arg = DeclareLaunchArgument('port', default_value='9090',
                                        description='Port number for ROS communication')

    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='test_rover/test_rover.urdf.xacro',
                                        description='URDF file name')

    allow_save_arg = DeclareLaunchArgument('allow_save', default_value='false',
                                        description='Enable saving files (default: false)')

    allow_overwrite_arg = DeclareLaunchArgument('allow_overwrite', default_value='false',
                                        description='Allow overwrite of existing files (default: false)')

    #### DEFINE PATHS ####
    # This will run the ROS# communication server, start up the URDF publisher, and then
    pkg_file_server2 = get_package_share_directory('file_server2')
    rosbridge_server_launch_file = PathJoinSubstitution(
        [pkg_file_server2, 'launch', 'ros_sharp_communication.launch.py'])

    urdf_launch_path = get_package_share_directory('urdf_launch')
    urdf_launch_launch_file = PathJoinSubstitution(
        [urdf_launch_path, 'launch', 'description.launch.py'])

    simulation = get_package_share_directory('simulation')
    custom_urdf_file_path = PathJoinSubstitution([
        simulation,
        "urdf",
        LaunchConfiguration('urdf_file')
    ])


    # Include ROS Sharp Communication launch file
    ros_sharp_communication_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbridge_server_launch_file]),
                launch_arguments={
                    'port': LaunchConfiguration('port'),
                    'allow_save': LaunchConfiguration('allow_save'),
                    'allow_overwrite': LaunchConfiguration('allow_overwrite')
                }.items()
        )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([urdf_launch_launch_file]),
        launch_arguments={
            'urdf_package': 'urdf_tutorial',
            'urdf_package_path': custom_urdf_file_path}.items()
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )


    return LaunchDescription([
        port_arg,
        urdf_file_arg,
        allow_save_arg,
        allow_overwrite_arg,
        ros_sharp_communication_launch,
        rover_launch,
        # joint_state_publisher_node
    ])
