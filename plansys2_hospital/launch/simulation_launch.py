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
    example_dir = get_package_share_directory('plansys2_hospital')
    
    
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('br2_tiago'),
            'launch',
            'sim.launch.py')))
    
    rviz_path = os.path.join(get_package_share_directory('hospital_navigation'), 'rviz_conf/configuracion.rviz')
    print(rviz_path)
    rviz_cmd=Node(
          package='rviz2', executable='rviz2', name="rviz2", output='screen', arguments=['-d'+str(rviz_path)]
        )
    
    rqt_path = os.path.join(get_package_share_directory('hospital_navigation'), 'rqt_conf/plansys2.perspective')
    rqt_plansys2 =Node(
          package='rqt_gui', executable='rqt_gui', name="rqt_gui", output='screen', arguments=[]
        )
    ld = LaunchDescription()


    #ld.add_action(gazebo_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(rqt_plansys2)
   
    return ld
