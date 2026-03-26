import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path Discovery
    repo_path = os.getenv('SIM_REPO_PATH', os.path.expanduser('~/nonlin_ctrl_lab/control_performance_comparison/Pure_Gazebo_SL_Harmonic'))
    px4_path = os.getenv('PX4_AUTOPILOT_PATH', os.path.expanduser('~/PX4-Autopilot'))
    
    # Target the build directory, NOT the source ROMFS
    px4_build_path = os.path.join(px4_path, 'build/px4_sitl_default')
    px4_bin = os.path.join(px4_build_path, 'bin/px4')
    px4_etc = os.path.join(px4_build_path, 'etc')

    # Discovery for Agent
    agent_exec = shutil.which('MicroXRCEAgent')

    world_arg = DeclareLaunchArgument('world', default_value='bare_quad.world')
    world_path = PythonExpression(["'", LaunchConfiguration('world'), "' if '", LaunchConfiguration('world'), "'.endswith('.sdf') else '", os.path.join(repo_path, 'worlds', ''), LaunchConfiguration('world'), "'"])

    # 2. Processes
    dds_agent = ExecuteProcess(
        cmd=[agent_exec, 'udp4', '-p', '8888'],
        output='screen'
    ) if agent_exec else LogInfo(msg="Agent not found.")

    px4_sitl = ExecuteProcess(
        # The key: pointing to 'etc' in the build folder
        cmd=[px4_bin, px4_etc, '-s', 'etc/init.d-posix/rcS', '-i', '0', '-d'],
        cwd=px4_build_path,
        env={
            'PATH': f"{os.path.join(px4_build_path, 'bin')}:{os.environ['PATH']}",
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_GZ_WORLD': world_path,
            'PX4_SIM_MODEL': 'custom_iris',
            'UXRCE_DDS_CFG': 'udp',
            'UXRCE_DDS_PRT': '8888'
        },
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        dds_agent,
        px4_sitl,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(repo_path, 'models')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(repo_path, 'plugin', 'build')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [f"-r ", world_path], 'gz_version': '8'}.items(),
        )
    ])