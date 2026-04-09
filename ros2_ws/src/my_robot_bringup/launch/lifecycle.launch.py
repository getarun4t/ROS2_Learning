from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    managed_node_name = "Python_peru"

    number_node = LifecycleNode(
        package = "lifecycle_py",
        executable = "number_publisher",
        name = managed_node_name,
        namespace=""
    )

    lifecycle_node_manager = Node(
        package = "lifecycle_py",
        executable = "lifecycle_node_manager",
        name = "manager",
        parameters=[
            {"managed_node_name": managed_node_name}
        ]
    )

    ld.add_action(number_node)
    ld.add_action(lifecycle_node_manager)

    return ld