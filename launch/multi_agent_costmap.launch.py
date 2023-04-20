#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia

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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Names of the robots
    robots = [
        {'name': 'agent1'},
        {'name': 'agent2'},
        {'name': 'agent3'}]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # lifecycle_map_node = ['map_server']

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bringup_dir = get_package_share_directory('pedsim_simulator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
    'autostart', default_value='true',
    description='Automatically startup the nav2 stack')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_house_demo_crowd.yaml'),
        description='Full path to map yaml file to load')

    declare_agent_1_params_file_cmd = DeclareLaunchArgument(
        'agent1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_param_agent_1.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_agent_2_params_file_cmd = DeclareLaunchArgument(
        'agent2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_param_agent_2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_agent_3_params_file_cmd = DeclareLaunchArgument(
        'agent3_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_param_agent_3.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_house_demo_crowd.yaml'),
        description='Full path to map yaml file to load')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")
        
        group = GroupAction([
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                          'launch',
                                                          'nav2_costmap_server.launch.py')),
              launch_arguments={'namespace': robot['name'],
                                'map': map_yaml_file,
                                'use_sim_time': use_sim_time,
                                'params_file': params_file,
                                'autostart': autostart,   
                                }.items()
          ),       
        ])
        nav_instances_cmds.append(group)
        # Nodes launching commands

    # start_map_server_cmd = launch_ros.actions.Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         output='screen',
    #         emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #         parameters=[{'yaml_filename': map_yaml_file}],
    #         remappings=remappings)

    # start_lifecycle_map_manager_cmd = launch_ros.actions.Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_map_manager',
    #         output='screen',
    #         emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #         parameters=[{'use_sim_time': use_sim_time,
    #                       'autostart': autostart,
    #                       'node_names': lifecycle_map_node}],
    #         remappings=remappings)

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_agent_1_params_file_cmd)
    ld.add_action(declare_agent_2_params_file_cmd)
    ld.add_action(declare_agent_3_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_lifecycle_map_manager_cmd)
    
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)
        print("MIP")

    return ld