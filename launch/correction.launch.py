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
    
    corrected_scan_topic_arg = DeclareLaunchArgument(
        'corrected_scan_topic',
        default_value='/corrected_laser_scan',
        description='Output corrected laser scan topic'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=os.path.join(pkg_dir, 'config', 'lidar_calibration.txt'),
        description='Path to calibration file'
    )
    
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Publish visualization markers'
    )
    
    intensity_threshold_arg = DeclareLaunchArgument(
        'intensity_threshold',
        default_value='200.0',
        description='Minimum intensity value to apply correction'
    )
    
    correction_node = Node(
        package='lidar_tape_correction',
        executable='correction_node',
        name='lidar_correction_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'corrected_scan_topic': LaunchConfiguration('corrected_scan_topic'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'publish_markers': LaunchConfiguration('publish_markers'),
            'intensity_threshold': LaunchConfiguration('intensity_threshold')
        }]
    )
    
    return LaunchDescription([
        scan_topic_arg,
        corrected_scan_topic_arg,
        calibration_file_arg,
        publish_markers_arg,
        intensity_threshold_arg,
        correction_node
    ])