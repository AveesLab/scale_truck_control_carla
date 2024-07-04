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
    
    # parameter files
        ros_param_file = os.path.join(
            get_package_share_directory('yolo_object_detection_ros2'),
            'config',
            'cam_params.yaml')
            
        yolo_param_file = os.path.join(
            get_package_share_directory('yolo_object_detection_ros2'),
            'config',
            'yolov3-tiny-custom.yaml')

        tracking_param_file = os.path.join(
                get_package_share_directory('object_tracking_ros2'),
                'config',
                'tracking.yaml'
        )

        fusing_param_file = os.path.join(
                get_package_share_directory('sensor_fusing_ros2'),
                'config',
                'fusing.yaml'
        )

    # start darknet and ros wrapper
        darknet_node=Node(
            package='yolo_object_detection_ros2',
            namespace='truck1',
            name='yolo_object_detection_node',
            executable='yolo_object_detection_ros2',
            output='screen',
            parameters = [yolo_param_file])


        tracking_node = Node(
                package = 'object_tracking_ros2',
                namespace = 'truck1',
                name = 'object_tracking_node',
                executable = 'object_tracking_ros2',
                output = 'screen',
                parameters = [tracking_param_file]
        )

        fusing_node = Node(
                package = 'sensor_fusing_ros2',
                namespace = 'truck1',
                name = 'sensor_fusing_node',
                executable = 'sensor_fusing_ros2',
                output = 'screen',
                parameters = [fusing_param_file]
        )
        ld = LaunchDescription()

        ld.add_action(darknet_node)
        ld.add_action(tracking_node)
        ld.add_action(fusing_node)
        return ld
