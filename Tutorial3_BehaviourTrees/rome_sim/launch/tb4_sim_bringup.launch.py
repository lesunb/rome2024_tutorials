# Adaoted from https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'], description='Auto start the nav2 stack.'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'], description='Launch tb3 description server'),
    DeclareLaunchArgument('use_respawn', default_value='true',
                          choices=['true', 'false'], description='Launch tb3 description server'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('localization', default_value='true',
                          choices=['true', 'false'], description='Start localization.'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'], description='Start nav2.'),
    DeclareLaunchArgument('params_file', default_value=f'{get_package_share_directory("rome_sim")}/params/nav2_params.yaml',
                          description='Full path to Nav2 params file.'),
    DeclareLaunchArgument('map',
                          default_value=f'{get_package_share_directory("rome_sim")}/maps/warehouse.yaml',
                          description='Full path to map yaml.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])

    #TODO Make UnB map and add the .sdf here
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('autostart', LaunchConfiguration('autostart')),
            ('description', LaunchConfiguration('description')),
            ('use_respawn', LaunchConfiguration('use_respawn')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('map', LaunchConfiguration('map')),
            ('nav2', LaunchConfiguration('nav2')),
            ('localization', LaunchConfiguration('localization')),
            ('params_file', LaunchConfiguration('params_file')),]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    return ld