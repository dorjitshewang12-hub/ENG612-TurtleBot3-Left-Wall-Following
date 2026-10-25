"""Launch the ENG612 lab arena in Gazebo Classic with a TurtleBot3 Burger.

Usage:
    export TURTLEBOT3_MODEL=burger
    ros2 launch eng612_wall_following lab_arena.launch.py
    ros2 launch eng612_wall_following lab_arena.launch.py x_pose:=-0.5 y_pose:=0.55

The arena walls are the Building Editor model in models/lab_arena_walls.
Default start pose matches the taped start position in the real arena:
robot 0.30 m from the north wall (wall on its LEFT), facing +x.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('eng612_wall_following')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_launch = os.path.join(tb3_gazebo, 'launch')

    world = os.path.join(pkg_share, 'worlds', 'lab_arena.world')
    models = os.path.join(pkg_share, 'models')
    # Let Gazebo find model://lab_arena_walls, keeping any existing paths.
    model_path = models + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Inner wall faces are at +/-0.915 m, so y = 0.915 - 0.30 = 0.615.
        DeclareLaunchArgument('x_pose', default_value='-0.5'),
        DeclareLaunchArgument('y_pose', default_value='0.615'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])
