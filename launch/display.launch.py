import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('motion_player')

    # Paths to URDF and RViz config
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'g1_custom_collision_29dof.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='Path to the URDF file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz config file'
    )

    motion_file_arg = DeclareLaunchArgument(
        'motion_file',
        description='Path to the motion pickle file (.pkl)'
    )

    bvh_file_arg = DeclareLaunchArgument(
        'bvh_file',
        default_value='',
        description='Path to the BVH file. If empty, auto-detects from motion_file path.'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Whether to loop the motion playback'
    )

    # Read URDF file content
    with open(default_urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Motion Player node (replaces joint_state_publisher_gui)
    motion_player_node = Node(
        package='motion_player',
        executable='motion_player',
        name='motion_player',
        output='screen',
        parameters=[{
            'motion_file': LaunchConfiguration('motion_file'),
            'bvh_file': LaunchConfiguration('bvh_file'),
            'loop': LaunchConfiguration('loop'),
        }]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        urdf_file_arg,
        rviz_config_arg,
        motion_file_arg,
        bvh_file_arg,
        loop_arg,
        robot_state_publisher_node,
        motion_player_node,
        rviz_node
    ])

