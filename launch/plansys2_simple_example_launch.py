# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2-hospital-k-bots')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/hospital_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_directly_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='move_directly_action_node',
        name='move_directly_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_through_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='move_through_action_node',
        name='move_through_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    drop_object_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='drop_object_node',
        name='drop_object_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    pick_object_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='pick_object_node',
        name='pick_object_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    open_door_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='open_door_node',
        name='open_door_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    close_door_cmd = Node(
        package='plansys2-hospital-k-bots',
        executable='close_door_node',
        name='close_door_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_directly_cmd)
    ld.add_action(move_through_cmd)
    ld.add_action(drop_object_cmd)
    ld.add_action(pick_object_cmd)
    ld.add_action(open_door_cmd)
    ld.add_action(close_door_cmd)
   
    return ld
