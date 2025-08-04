from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="mathi_pkg",
        executable="number_publisher"
        remappings=[("number", "/new_number")]
        parameters=[
            {"number" : 12},
            {"timer_period" : 2.2}
        ]
    )

    number_counter = Node(
        package="mathi_pkg",
        executable="number_counter"
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)

    return ld