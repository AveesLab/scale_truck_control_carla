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

    ###############
    # Lidar param #
    ###############
#    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
#    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') 
#    frame_id = LaunchConfiguration('frame_id', default='laser')
#    inverted = LaunchConfiguration('inverted', default='false')
#    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
#    scan_mode=LaunchConfiguration('scan_mode', default='DenseBoost')#Standard,DenseBoost

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') #for A3 is 256000
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Stability')


    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')                 
    
    lane_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'LV.yaml')                 


    object_node=Node(
            package="object_detection_ros2",
            namespace='LV',
            executable="object_detection_ros2_node",
            output={
            'stdout': 'screen',
            'stderr': 'screen',
            })

    lane_detection_node=Node(
            package='lane_detection_ros2',
            namespace='LV',
            name='LaneDetector', # .yaml에 명시.
            executable='lane_detect_node',
            output='screen',
            parameters = [lane_param_file])


    ld = LaunchDescription()
    
    ld.add_action(lane_detection_node)
   # ld.add_action(object_node)

    return ld


