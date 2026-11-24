from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    megoldas = Node(
        package='mobil_robot',
        executable='megoldas',
        name='megoldas',
        output='screen'
    )

    return LaunchDescription([megoldas])