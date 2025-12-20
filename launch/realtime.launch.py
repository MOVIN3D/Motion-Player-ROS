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
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot.rviz')

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

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='11235',
        description='UDP port to listen for OSC mocap data'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='unitree_g1',
        description='Target robot type (unitree_g1 or unitree_g1_with_hands)'
    )

    human_height_arg = DeclareLaunchArgument(
        'human_height',
        default_value='1.75',
        description='Human height in meters for scaling'
    )

    skeleton_offset_x_arg = DeclareLaunchArgument(
        'skeleton_offset_x',
        default_value='1.0',
        description='X offset to place skeleton beside robot'
    )

    offset_to_ground_arg = DeclareLaunchArgument(
        'offset_to_ground',
        default_value='true',
        description='Whether to offset the motion to place lowest foot at ground'
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

    # Real-time Motion Player node
    motion_player2_node = Node(
        package='motion_player',
        executable='motion_player2',
        name='motion_player2',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'robot_type': LaunchConfiguration('robot_type'),
            'human_height': LaunchConfiguration('human_height'),
            'skeleton_offset_x': LaunchConfiguration('skeleton_offset_x'),
            'offset_to_ground': LaunchConfiguration('offset_to_ground'),
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
        port_arg,
        robot_type_arg,
        human_height_arg,
        skeleton_offset_x_arg,
        offset_to_ground_arg,
        robot_state_publisher_node,
        motion_player2_node,
        rviz_node
    ])

