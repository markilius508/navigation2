import os
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable
import xacro

def generate_launch_description():
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    tutorial_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    bringup_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # Process the world xacro file
    world_sdf_xacro = os.path.join(tutorial_dir, "worlds", "tb3_sonoma_raceway.sdf.xacro")
    world_xml = xacro.process_file(world_sdf_xacro).toxml()
    robot_sdf = os.path.join(sim_dir, "urdf", "gz_waffle_gps.sdf.xacro")

    # Get robot description from URDF
    urdf_path = os.path.join(sim_dir, "urdf", "turtlebot3_waffle_gps.urdf")
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # Save to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        f.write(world_xml)
        world_sdf = f.name

    # --- NEW GAZEBO SPLIT LAUNCH STYLE ---
    
    # Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [f"-r -s -v 2 {world_sdf}"], 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Gazebo Client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v 2', 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # -------------------------------------

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb3_gps.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_sdf": robot_sdf,
            "x_pose": "2.0",
            "y_pose": "-2.5",
            "z_pose": "0.33",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
        }.items(),
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "robot_description": robot_description}
        ],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'models')
    )
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(os.path.join(bringup_dir)).parent.resolve())
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    
    # Add server and client separately
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot)
    ld.add_action(start_robot_state_publisher_cmd)
    
    # ld.add_action(TimerAction(
    #     period=3.0,
    #     actions=[spawn_robot]
    # ))

    # ld.add_action(TimerAction(
    #     period=10.0,
    #     actions=[start_robot_state_publisher_cmd]
    # ))
    
    return ld