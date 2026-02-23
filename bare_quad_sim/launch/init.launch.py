import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Launch Gazebo with PX4 SITL and ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    PX4_AIRFRAMES_DIR = os.path.join(HOME, 'PX4-Autopilot/build/px4_sitl_default/etc/init.d/airframes')
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # world path
    world_path = os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world') # modify gazebo classic world file

    # model path
    setup_package_path = get_package_share_directory('setup')
    model_path = os.path.join(setup_package_path, 'models/px4vision/px4vision.sdf')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        # Set environment variables for Gazebo and PX4
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            # Combine PX4 models and custom models
            os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models') + ':' +
            os.path.join(setup_package_path, 'models')
        ),

        # Set Drone ID and Estimator Type
        DeclareLaunchArgument('drone_id', default_value='0', description='PX4 Drone ID'), 
        SetEnvironmentVariable('PX4_SIM_DRONE_ID', LaunchConfiguration('drone_id')),
        SetEnvironmentVariable('PX4_MAV_SYS_ID', LaunchConfiguration('drone_id')),
        DeclareLaunchArgument('est', default_value='ekf2', description='PX4 Estimator Type'),
        SetEnvironmentVariable('PX4_ESTIMATOR', LaunchConfiguration('est')),
        
        # Declare launch arguments
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('model', default_value=model_path),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('paused', default_value='true'), # start Gazebo in paused mode
        DeclareLaunchArgument('initial_sim_time', default_value=''), # start Gazebo with initial sim time

        # Launch Gazebo server and client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': 'true', 
                'pause': LaunchConfiguration('paused'),
                'initial_sim_time': LaunchConfiguration('initial_sim_time')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),

        # Spawn sls
        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model'),
                '--model-name', 'px4vision_sls', 
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'
        ),

        # Start PX4 SITL
        ExecuteProcess(
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s', HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            shell=True,
            prefix="xterm -hold -e",
            output='screen'
        ),

        # Start Micro XRCE-DDS Agent for ROS2 <--> PX4 communication
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'),
    ])
