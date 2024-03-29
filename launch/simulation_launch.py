import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    # ----------------------------------------------------------
    # PACKAGE
    # ----------------------------------------------------------
    package_name = 'binocular_laser_profile_scanner'
    package_path = get_package_share_directory(package_name)
    
    # ----------------------------------------------------------
    # GAZEBO WORLD
    # ----------------------------------------------------------
    world = os.path.join(package_path, 'worlds', 'my_empty_world.world')

    # ----------------------------------------------------------
    # RVIZ CONFIG
    # ----------------------------------------------------------
    rviz_config = os.path.join(package_path, 'config', 'config.rviz')

    # ----------------------------------------------------------
    # ROBOT DESCRIPTION(XACRO)
    # ----------------------------------------------------------
    urdf_file_name = package_name + '.xacro'
    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # ----------------------------------------------------------
    # DECLARED ARGUMENTS
    # ----------------------------------------------------------
    

    gazebo_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world,
        description="Full path to world model file to load",
    )

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_gui = LaunchConfiguration('gui')
    use_gui_cmd = DeclareLaunchArgument(
        'gui', 
        default_value='false',
        description='Use gazebo gui if true')

    declared_arguments = [
        gazebo_world_cmd,
        use_sim_time_cmd,
        use_gui_cmd
    ]

    # ----------------------------------------------------------
    # EXECUTE PROCESSES
    # ----------------------------------------------------------
    execute_gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            world,
        ],
        output="screen"
    )
    
    execute_gazebo_client = ExecuteProcess(
        cmd=["gzclient"], 
        output="screen",
        condition=IfCondition(use_gui)
    )
    
    execute_processes = [
        execute_gazebo_server,
        execute_gazebo_client
    ]

    # ----------------------------------------------------------
    # NODES
    # ----------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'binocular_laser_profile_scanner', '-unpause'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    nodes = [
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        spawn_entity,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + execute_processes + nodes)