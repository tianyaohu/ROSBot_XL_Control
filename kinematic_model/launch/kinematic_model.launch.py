from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='Turn on/off sim time setting'
        ),

        Node(
            package='wheel_velocities_publisher',
            executable='wheel_vel_pub_node',
            name='wheel_vel_pub_node',
            output='screen',
            parameters=[{'debug_topics': True}],
        ),

        Node(
            package='kinematic_model',  
            executable='kinematic_model_node',  
            name='kinematic_model_node',
            output='screen',
            parameters=[{'debug_topics': True}],
        ),
    ])