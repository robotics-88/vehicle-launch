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

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server', 'planner_server']
    use_sim_time = True
    autostart = True
    save_map_timeout = 2.0
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65

    costmap_config_path = os.path.join(
        get_package_share_directory('vehicle_launch'),
        'config',
        'decco_costmap.yaml'
    )

    map_server_config_path = os.path.join(
        get_package_share_directory('vehicle_launch'),
        'config',
        'decco_costmap_server.yaml'
    )

    planner_server_config_path = os.path.join(
        get_package_share_directory('vehicle_launch'),
        'config',
        'nav2_params.yaml'
    )

    # Nodes launching commands
    start_map_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        output='log',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[map_server_config_path],
        arguments=['--ros-args', '--log-level', 'error']
    )

    start_costmap_cmd = launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='log',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[planner_server_config_path],
        arguments=['--ros-args', '--log-level', 'error'],
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='log',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
        arguments=['--ros-args', '--log-level', 'error'],
    )

    ld = LaunchDescription()

    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld