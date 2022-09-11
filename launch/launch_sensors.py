# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('i2c_sensors'),
        'config',
        'params.yaml'
        )

    i2c_sensors = launch_ros.actions.Node(package='i2c_sensors',
                                           name = 'i2c_sensors_node',
                                           executable='i2c_sensors_node', 
                                           parameters=[config],
                                           output='screen')


    return launch.LaunchDescription([
        i2c_sensors
    ])
