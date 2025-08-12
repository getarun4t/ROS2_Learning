from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    # Process xacro file
    urdf_path = os.path.join(
        os.getenv('HOME'), 'Kannan', 'src', 'my_robot_description', 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    rviz_config_file = os.path.join(
        os.getenv('HOME'), 'Kannan', 'src', 'my_robot_description', 'rviz', 'urdf_config.rviz')

    gazebo_config_path = os.path.join(
        os.getenv('HOME'), 'Kannan', 'src', 'my_gazebo_robot_bringup', 'config', 'gazebo_bridge.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            arguments=['empty.sdf', '-r'],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gazebo_config_path}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
