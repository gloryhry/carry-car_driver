from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='carry_along_car_driver',
            executable='rs232_driver',
            name='rs232_driver_node',
            parameters=[
                {'port': '/dev/tty'},
                {'baudrate': 9600},
                {'timeout': 1000},
                {'wheel_width': 0.4},
                {'wheel_diameter': 0.2}
            ]
        )
    ])