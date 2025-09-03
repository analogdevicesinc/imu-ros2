# Copyright 2025 Analog Devices, Inc.
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
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    iio_context_string_arg = DeclareLaunchArgument(
        'iio_context_string',
        description='The URI of the processing unit to which the IMU is connected to (e.g: ip:192.168.2.1).',
    )
    imu_device_name_arg = DeclareLaunchArgument(
        'imu_device_name',
        description='The name of the IMU device to be used (use lowercase and a dash, e.g., adis16545-3).',
    )
    measured_data_topic_selection_arg = DeclareLaunchArgument(
        'measured_data_topic_selection',
        description=(
            'Selects the topic mode for sending IMU data:\n'
            '\t0: measured data is published on /velangtempdata topic - not available for adis1646x; sampling is performed on each data ready impulse\n'
            '\t1: measured data is published on /accelgyrotempdata topic; sampling is performed on each data ready impulse\n'
            '\t2: measured data is published on /imu topic; sampling performed on each data ready impulse\n'
            '\t3: measured data is published on /imufullmeasureddata topic (default); sampling is performed by polling the data registers without taking into consideration the data ready impulse\n'
        ),
        default_value='2',
        choices=['0', '1', '2', '3'],
    )
    diag_data_enable_arg = imu_device_name_arg = DeclareLaunchArgument(
        'diag_data_enable',
        description='Whether to enable the publisher of IMU diagnostic data.',
        default_value='false'
    )
    ident_data_enable_arg = imu_device_name_arg = DeclareLaunchArgument(
        'ident_data_enable',
        description='Whether to enable the publisher of IMU identification data.',
        default_value='false'
    )

    iio_context_string = LaunchConfiguration('iio_context_string')
    imu_device_name = LaunchConfiguration('imu_device_name')
    measured_data_topic_selection = LaunchConfiguration('measured_data_topic_selection')
    diag_data_enable = LaunchConfiguration('diag_data_enable')
    ident_data_enable = LaunchConfiguration('ident_data_enable')

    rviz_param = launch.substitutions.LaunchConfiguration(
        'rviz_param',
        default=os.path.join(
            get_package_share_directory('adi_imu'),
            'rviz',
            'imu_tof_fusion.rviz'))

    adi_imu_node = launch_ros.actions.Node(
        package='adi_imu',
        executable='adi_imu_node',
        parameters=[
            {'iio_context_string': iio_context_string},
            {'imu_device_name': imu_device_name},
            {'measured_data_topic_selection': measured_data_topic_selection},
            {'diag_data_enable': diag_data_enable},
            {'ident_data_enable': ident_data_enable},
        ],
        remappings=[('/imu', '/imu/data_raw')],
        output='screen'
    )

    imu_filter_madgwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[
            {'use_mag': False},
        ],
        output='screen'
    )

    tof_node = launch_ros.actions.Node(
        package='tof_ros2cpp',
        executable='tof_ros2cpp_node',
        arguments=['ip=10.42.0.1', 'config_file=config/config_adsd3500_adsd3100.json', 'mode=3'],
        output='screen'
    )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu', 'aditof_xyz_img']
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param]
    )

    return launch.LaunchDescription([
        iio_context_string_arg,
        imu_device_name_arg,
        measured_data_topic_selection_arg,
        diag_data_enable_arg,
        ident_data_enable_arg,
        adi_imu_node,
        imu_filter_madgwick_node,
        tof_node,
        tf,
        rviz,
    ])
