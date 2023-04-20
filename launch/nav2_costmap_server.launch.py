#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    bringup_dir = get_package_share_directory('pedsim_simulator')
    
    lifecycle_bt_node = ['planner_server', 'controller_server']
    lifecycle_map_node = ['map_server']

    remappings = [('/tf', 'tf'),
                  # ('/tf', '/' + namespace.perform(context) + '/tf'),
                  ('/tf_static', 'tf_static')
                  ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=os.path.join(bringup_dir, 'params', 'nav2_param_agent_1.yaml'),
      description='Full path to the ROS2 parameters file to use for all launched nodes')
    
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

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Nodes launching commands
    start_planner_server_cmd = launch_ros.actions.Node(
      package='nav2_planner',
      executable='planner_server',
      output='screen',
      parameters=[configured_params],
      remappings=remappings,
      namespace=namespace)

    start_controller_server_cmd = launch_ros.actions.Node(
      package='nav2_controller',
      executable='controller_server',
      output='screen',
      parameters=[configured_params],
      remappings=remappings,
      namespace=namespace)

    start_map_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'yaml_filename': map_yaml_file}],
            namespace=namespace,
            remappings=remappings)

    start_lifecycle_map_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_map_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'node_names': lifecycle_map_node}],
            namespace=namespace,
            remappings=remappings)

    start_lifecycle_bt_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'node_names': lifecycle_bt_node}],
            namespace=namespace,
            remappings=remappings)

    start_pedsim_cmd = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                          'launch',
                                                          'house_demo_launch.py')),
              launch_arguments={'namespace': namespace,
                                }.items()
          )
   
    return [
      declare_namespace_cmd,
      declare_use_sim_time_cmd,
      declare_params_file_cmd,
      declare_autostart_cmd,
      declare_map_yaml_cmd,
      start_map_server_cmd,
      start_controller_server_cmd,
      start_planner_server_cmd,
      start_lifecycle_map_manager_cmd,
      start_lifecycle_bt_manager_cmd,
      start_pedsim_cmd
    ]

def generate_launch_description():
    opaque_function_cmd = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_map_yaml_cmd)
    
    # ld.add_action(start_planner_server_cmd)
    # ld.add_action(start_controller_server_cmd)
    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_lifecycle_map_manager_cmd)
    # ld.add_action(start_lifecycle_bt_manager_cmd)
    ld.add_action(opaque_function_cmd)

    return ld