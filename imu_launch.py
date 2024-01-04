from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodelet manager
        Node(
            package='nodelet',
            executable='nodelet',
            name='imu_manager',
            output='screen',
            arguments=['manager']
        ),
        
        # IMU Driver
        Node(
            package='nodelet',
            executable='nodelet',
            name='ImuFilterNodelet',
            output='screen',
            arguments=['load', 'imu_filter_madgwick/ImuFilterNodelet', 'imu_manager']
            # Add parameters if needed using the 'parameters' argument
            # parameters=[{'publish_tf': False}]
        ),
    ])
