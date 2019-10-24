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
        get_package_share_directory('handeye_target_detection'), 
            'launch', 'pose_estimation.yaml'
    )

    rviz = os.path.join(
        get_package_share_directory('handeye_target_detection'), 
            'cfg', 'handeye.rviz'
    )

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='handeye_target_detection', node_executable='pose_estimation', 
            output='screen', arguments=['__params:='+yaml]),

        launch_ros.actions.Node(
            package='rviz2', node_executable='rviz2', 
            output='screen', arguments=['-d', rviz]),
    ])