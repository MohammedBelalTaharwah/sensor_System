import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('sensor_fusion_system')
    
    # المسار الافتراضي لملف الإعدادات
    default_params_file = os.path.join(pkg_dir, 'config', 'sensor_params.yaml')

    # تعريف متغير لملف الإعدادات
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the parameters file to use'
    )
    params_file = LaunchConfiguration('params_file')

    # --- تعريف النُوَد الأربعة التي لديك ---
    
    sensor_publisher_node = Node(
        package='sensor_fusion_system',
        executable='sensor_publisher',
        name='sensor_publisher_node',
        parameters=[params_file],
        output='screen'
    )

    data_logger_node = Node(
        package='sensor_fusion_system',
        executable='data_logger',
        name='data_logger_node',
        parameters=[params_file],
        output='screen'
    )

    alert_system_node = Node(
        package='sensor_fusion_system',
        executable='alert_system',
        name='alert_system_node',
        parameters=[params_file],
        output='screen'
    )

    sensor_fusion_node = Node(
        package='sensor_fusion_system',
        executable='sensor_fusion',
        name='sensor_fusion_node',
        parameters=[params_file],
        output='screen'
    )

    # --- !! هذا هو النود الجديد الذي أضفناه !! ---
    
    visualization_node = Node(
        package='sensor_fusion_system',
        executable='visualizer', # هذا هو الاسم من 'entry_points'
        name='visualization_node',
        output='screen'
        # هذا النود لا يحتاج ملف الإعدادات (params_file)
    )

    # --- إرجاع القائمة الكاملة بـ 5 نُوَد ---
    return LaunchDescription([
        params_file_arg,
        sensor_publisher_node,
        data_logger_node,
        alert_system_node,
        sensor_fusion_node,
        visualization_node  # تمت إضافته هنا
    ])