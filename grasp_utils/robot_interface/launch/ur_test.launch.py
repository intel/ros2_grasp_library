# Copyright (c) 2019 Intel Corporation. All Rights Reserved
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

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # .yaml file for configuring the parameters
    yaml = os.path.join(
        get_package_share_directory('robot_interface'), 
            'launch', 'ur_test.yaml'
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            "move",
            default_value=["true"],
            description="If using the move command test"
        ),

        launch_ros.actions.Node(
            package='robot_interface', 
            node_executable='ur_test_state_publish', 
            output='screen', arguments=['__params:='+yaml],
            condition=launch.conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("move"))),

        launch_ros.actions.Node(
            package='robot_interface', 
            node_executable='ur_test_move_command', 
            output='screen', arguments=['__params:='+yaml],
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("move"))),        
    ])