#!/usr/bin/env python3

# Copyright 2026 LiveKit
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

"""Launch the pan/tilt servo driver (pan_tilt_driver).

Runs on the node's declared parameter defaults unless a params_file is
provided (bringup passes its shared cobra_flex.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)

    parameters = []
    if params_file:
        parameters.append(params_file)
    if serial_port:
        parameters.append({'serial_port': serial_port})

    return [
        Node(
            package='cobra_flex_pan_tilt',
            executable='pan_tilt_driver',
            name='pan_tilt_driver',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
                              description='Parameter file for the pan/tilt '
                                          'driver; empty runs on code defaults.'),
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Override the servo bus serial port; '
                                          'empty keeps the params_file/default '
                                          'value.'),
        OpaqueFunction(function=_launch_setup),
    ])
