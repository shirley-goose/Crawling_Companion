from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1) Include the TurtleBot3 GIX hardware launch
    tb3_gix_bringup_share = get_package_share_directory('turtlebot3_gix_bringup')
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb3_gix_bringup_share, 'launch', 'hardware.launch.py'])
        )
    )

    # 2) USB camera node with params and remap
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 360,
        }],
        remappings=[
            ('/image_raw', '/camera/rgb/image_raw'),
        ],
    )

    # 3) Crawling companion main node
    crawling_companion = Node(
        package='crawling_companion',
        executable='crawling_companion',
        name='crawling_companion',
        output='screen',
    )

    return LaunchDescription([
        hardware_launch,
        usb_cam,
        crawling_companion,
    ])
