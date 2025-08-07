from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_tape_correction')
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/bot_sensor/lidar_front/laser_scan',
        description='Input laser scan topic'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=os.path.join(pkg_dir, 'config', 'lidar_calibration.txt'),
        description='Path to save calibration file'
    )
    
    intensity_threshold_arg = DeclareLaunchArgument(
        'intensity_threshold',
        default_value='200.0',
        description='Minimum intensity value to consider as high-reflectance'
    )
    
    calibration_node = Node(
        package='lidar_tape_correction',
        executable='calibration_node',
        name='lidar_calibration_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'intensity_threshold': LaunchConfiguration('intensity_threshold'),
            'collection_window_size': 10,
            'high_intensity_angle_range': 5.0
        }]
    )
    
    return LaunchDescription([
        scan_topic_arg,
        calibration_file_arg,
        intensity_threshold_arg,
        calibration_node
    ])