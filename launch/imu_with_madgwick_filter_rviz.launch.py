import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_param = launch.substitutions.LaunchConfiguration(
        'rviz_param',
        default=os.path.join(
            get_package_share_directory('adi_imu'),
            'rviz',
            'imu_with_madgwick_filter_rviz.rviz'))

    adi_imu_node = launch_ros.actions.Node(
        package='adi_imu',
        executable='adi_imu_node',
        parameters=[{'measured_data_topic_selection': 2},
                    # the IP address of the processing unit to which the IMU is connected to
                    {'iio_context_string': "ip:0.0.0.0"},],
        remappings=[('/imu','/imu/data_raw')],
        output='screen'
        )

    imu_filter_madgwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{'use_mag': False},],
        output='screen'
        )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param]
        )


    return launch.LaunchDescription([
        adi_imu_node,
        imu_filter_madgwick_node,
        rviz,
            ])
