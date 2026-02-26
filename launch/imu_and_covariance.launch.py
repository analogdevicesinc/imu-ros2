# Copyright 2026 Analog Devices, Inc.
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


import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get config file path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('adi_imu'),
            'config',
            'imu_covariance_config.yaml'
        ]),
        description='Path to the YAML configuration file'
    )
    
    config_file = LaunchConfiguration('config_file')
    
    # Keep required args without defaults (user must provide)
    iio_context_string_arg = DeclareLaunchArgument(
        'iio_context_string',
        description='The URI of the IMU (e.g: ip:192.168.2.1).',
    )
    imu_device_name_arg = DeclareLaunchArgument(
        'imu_device_name', 
        description='IMU device name (e.g., adis16545-3).',
    )
    
    adi_imu_node = launch_ros.actions.Node(
        package='adi_imu',
        executable='adi_imu_node',
        parameters=[
            config_file,  # Load YAML config first
            {'iio_context_string': LaunchConfiguration('iio_context_string')},
            {'imu_device_name': LaunchConfiguration('imu_device_name')},
        ],
        remappings=[('/imu', '/imu/data_raw')],
        output='screen'
    )
    
    return launch.LaunchDescription([
        config_file_arg,
        iio_context_string_arg,
        imu_device_name_arg,
        adi_imu_node,
    ])