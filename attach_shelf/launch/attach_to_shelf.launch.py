import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    package_description = "attach_shelf"
    # args that can be set from the command line or a default will be used
    # TextSubstitution(text="0.0") Will only evaluate that in execution time.
    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.3" )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90" )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false" )
    
    
    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f =  LaunchConfiguration('final_approach')
   
   # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'default.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    
    approach_service_node = Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen',
            name='approach_service_server_node',
            )
    
    pre_approach_node_v2 = Node(
            package='attach_shelf',
            executable='pre_approach_v2_node',
            output='screen',
            name='pre_approach_v2_node',
            parameters=[
                {'obstacle': obstacle_f,
                'degrees' : degrees_f,
                "final_approach" : final_approach_f,
                }
            ]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        rviz_node,
        approach_service_node,
        pre_approach_node_v2,    
    ])
