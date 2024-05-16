#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Declare the launch argument for the truck name
    declare_truck_name_arg = DeclareLaunchArgument(
        'truck_name',  # Name of the launch argument
        default_value='truck0',  # Default value if none provided
        description='Name of the truck'
    )

    base_directory = os.path.dirname(os.path.realpath(__file__)) 
    config_directory = os.path.join(base_directory, '../../', 'config') 
    
    ros_param_file = os.path.join(config_directory,'config.yaml')                 
    lane_param_file = os.path.join(config_directory,'LV.yaml')                 

    # Node #
    lane_detection_node=Node(
            package='sliding_window_lane_detection',
            namespace='truck0',
            name='LaneDetector', # .yaml에 명시.
            executable='lane_detect_node',
            output='screen',
            parameters = [lane_param_file])
    
    lane_keeping_node=Node(
            package='lane_keeping',
            namespace='truck0',
            name='LaneKeeping', # .yaml에 명시.
            executable='lane_keeping_node',
            output='screen',
            parameters = [lane_param_file])
            
    object_node=Node(
            package="obstacle_detection",
            namespace='truck0',
            executable="obstacle_detection_node",
            output={
            'stdout': 'screen',
            'stderr': 'screen',
            })

    speed_control_node=Node(
            package='speed_control', 
            namespace='truck0', 
            name='VelocityControl', 
            executable='speed_control_node', 
            output='screen')
    v2v_node=Node(
            package='v2v', 
            namespace='truck0', 
            name='V2Vnetwork', 
            executable='v2v_node', 
            output='screen',
            parameters=[{'truck_name': LaunchConfiguration('truck_name')}])

    plan_node=Node(
            package='planner', 
            namespace='truck0', 
            name='Planning', 
            executable='planner_node', 
            output='screen',
            parameters=[{'truck_name': LaunchConfiguration('truck_name')}])
    interface_node=Node(
            package='interface', 
            namespace='truck0', 
            name='Interface', 
            executable='interface_node', 
            output='screen')

    ld = LaunchDescription([
        declare_truck_name_arg,  # Add the launch argument action
        lane_detection_node,
        lane_keeping_node,
        object_node,
        speed_control_node,
        v2v_node,
        plan_node,
        interface_node
    ])
    return ld


