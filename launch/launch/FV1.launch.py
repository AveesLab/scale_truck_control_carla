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
        default_value='truck1',  # Default value if none provided
        description='Name of the truck'
    )

    base_directory = os.path.dirname(os.path.realpath(__file__)) 
    config_directory = os.path.join(base_directory, '../../', 'config') 
    
    ros_param_file = os.path.join(config_directory,'config.yaml')                 
    lane_param_file = os.path.join(config_directory,'FV1.yaml')                 
    yolo_param_file = os.path.join(config_directory,'yolo.yaml')
    fusion_param_file = os.path.join(config_directory,'fusing.yaml')  
    # Node #
    lane_detection_node=Node(
            package='ultra_fast_lane_detection',
            namespace='truck1',
            name='LaneDetector', # .yaml에 명시.
            executable='lane_detect_node',
            output='screen',
            parameters = [lane_param_file])
    
    lane_keeping_node=Node(
            package='lane_keeping',
            namespace='truck1',
            name='LaneKeeping', # .yaml에 명시.
            executable='lane_keeping_node',
            output='screen',
            parameters = [lane_param_file])
            
    predict_lc_node=Node(
            package="carla_gnss_converter",
            namespace='truck1',
            executable="gnss_converter",
            output={
            'stdout': 'screen',
            'stderr': 'screen',
            })

    object_node=Node(
            package="obstacle_detection",
            namespace='truck1',
            executable="obstacle_detection_node",
            output={
            'stdout': 'screen',
            'stderr': 'screen',
            })

    cluster_node1 = Node(
            package="euclidean_cluster",
            namespace="truck1",
            executable="euclidean_cluster_node",
            name="euclidean_cluster_node",
            parameters = [ {'sub_topic_name': 'front_radar'}],
            output='screen'
    )

    cluster_node2 = Node(
            package="euclidean_cluster",
            namespace="truck1",
            executable="euclidean_cluster_node",
            name="euclidean_cluster_node_right",
            parameters = [ {"sub_topic_name": 'right_radar'} , {"pub_topic_name": 'right_clustered_radar_points'}],
            output='screen'
    )

    cluster_node3 = Node(
            package="euclidean_cluster",
            namespace="truck1",
            executable="euclidean_cluster_node",
            name="euclidean_cluster_node_left",
            parameters = [ {"sub_topic_name": 'left_radar'} , {"pub_topic_name": 'left_clustered_radar_points'}],
            output='screen'
    )

    speed_control_node=Node(
            package='speed_control', 
            namespace='truck1', 
            name='speed_control_node', 
            executable='speed_control_node', 
            output='screen')
    v2v_node=Node(
            package='v2v', 
            namespace='truck1', 
            name='v2v', 
            executable='v2v_node', 
            output='screen',
            parameters=[{'truck_name': LaunchConfiguration('truck_name')}])

    plan_node=Node(
            package='planner', 
            namespace='truck1', 
            name='planner', 
            executable='planner_node', 
            output='screen',
            parameters=[{'truck_name': LaunchConfiguration('truck_name')}])
    plan_node_wo=Node(
            package='plannerwo', 
            namespace='truck1', 
            name='plannerwo', 
            executable='planner_node_wo', 
            output='screen',
            parameters=[{'truck_name': LaunchConfiguration('truck_name')}])
    tracking_node=Node(
            package='object_tracking_ros2',
            namespace='truck1',
            name='tracking',
            executable='object_tracking_ros2',
            output='screen'
    )

    yolo_node=Node(
            package='yolo_object_detection_ros2',
            namespace='truck1',
            name='yolo',
            executable='yolo_object_detection_ros2',
            output='screen',
            parameters = [yolo_param_file]
    )

    fusion_node=Node(
            package='sensor_fusing_ros2',
            namespace='truck1',
            name='fusion',
            executable='sensor_fusing_ros2',
            output='screen',
    )

    test_fusion_node=Node(
            package='test_fusion_node',
            namespace='truck1',
            name='fusion2',
            executable='test_fusion_node',
            output='screen',
    )

    ld = LaunchDescription([
        declare_truck_name_arg,  # Add the launch argument action
        lane_detection_node,
        lane_keeping_node,
        #object_node,
        cluster_node1,
        #cluster_node2,
        #cluster_node3,
        speed_control_node,
        v2v_node,
        plan_node_wo,
        #tracking_node,
        yolo_node,
        test_fusion_node,
        predict_lc_node
        #fusion_node
    ])
    return ld


