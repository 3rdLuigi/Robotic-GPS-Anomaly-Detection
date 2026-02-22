from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='gps_anomaly_sim',
            executable='sensor_noise_node',
            name='sensor_noise_node'
        )
    ])