#!/usr/bin/env python3
"""
Launch file for Sensor Fusion System
Launches all nodes with proper configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for sensor fusion system"""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    alert_cooldown_arg = DeclareLaunchArgument(
        'alert_cooldown',
        default_value='5.0',
        description='Cooldown period between same alerts (seconds)'
    )
    
    # Sensor Publisher Node
    sensor_publisher = Node(
        package='sensor_fusion_system',
        executable='sensor_publisher',
        name='sensor_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_rate': 10.0,
            'gps_rate': 1.0,
            'temp_rate': 2.0,
            'baro_rate': 2.0,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Data Logger Node
    data_logger = Node(
        package='sensor_fusion_system',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Alert System Node
    alert_system = Node(
        package='sensor_fusion_system',
        executable='alert_system',
        name='alert_system',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'alert_cooldown': LaunchConfiguration('alert_cooldown'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Sensor Fusion Node
    sensor_fusion = Node(
        package='sensor_fusion_system',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'filter_window': 5,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Launch information
    launch_info = LogInfo(
        msg='Starting Sensor Fusion System with all nodes...'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(log_level_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(alert_cooldown_arg)
    
    # Add launch info
    ld.add_action(launch_info)
    
    # Add nodes
    ld.add_action(sensor_publisher)
    ld.add_action(data_logger)
    ld.add_action(alert_system)
    ld.add_action(sensor_fusion)
    
    return ld