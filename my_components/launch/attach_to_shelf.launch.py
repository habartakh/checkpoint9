import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description with multiple components.
    """
    
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='attach_server'),
            ],
            output='screen',
    )

    # RVIZ Configuration
    package_description = "my_components"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'default.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    return launch.LaunchDescription([
            container,
            rviz_node,
            ])