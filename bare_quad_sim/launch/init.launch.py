import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('bare_quad_sim')
    repo_path = os.getenv('SIM_REPO_PATH', os.path.expanduser('~/nonlin_ctrl_lab/control_performance_comparison/Pure_Gazebo_SL_Harmonic'))
    px4_path = os.getenv('PX4_AUTOPILOT_PATH', os.path.expanduser('~/PX4-Autopilot'))

    models_path = os.path.join(repo_path, 'models')
    plugins_path = os.path.join(repo_path, 'plugin', 'build')
    
    px4_build_path = os.path.join(px4_path, 'build/px4_sitl_default')
    px4_bin = os.path.join(px4_build_path, 'bin/px4')
    px4_etc = os.path.join(px4_build_path, 'etc')

    agent_exec = shutil.which('MicroXRCEAgent')

    world_arg = DeclareLaunchArgument('world', default_value='bare_quad.world')
    world_path = PythonExpression(["'", LaunchConfiguration('world'), "' if '", LaunchConfiguration('world'), "'.endswith('.sdf') else '", os.path.join(pkg_share, 'worlds', ''), LaunchConfiguration('world'), "'"])

    dds_agent = ExecuteProcess(
        cmd=[agent_exec, 'udp4', '-p', '8888'],
        output='screen'
    ) if agent_exec else LogInfo(msg="Agent not found.")

    px4_models_path = os.path.join(px4_path, 'Tools/simulation/gz/models')
    combined_models_path = f"{models_path}:{px4_models_path}"

    px4_env = os.environ.copy()
    px4_env.update({
        'PATH': f"{os.path.join(px4_build_path, 'bin')}:{os.environ.get('PATH', '')}",
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_GZ_WORLD': world_path,
        'PX4_SIM_MODEL': 'gz_x500', 
        'UXRCE_DDS_CFG': 'udp',
        'UXRCE_DDS_PRT': '8888',
        'PX4_PARAM_NAV_DLL_ACT': '0',
        'PX4_PARAM_NAV_RCL_ACT': '0',
        'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
        'PX4_PARAM_CBRK_USB_CHK': '197848',
        'PX4_PARAM_COM_ARM_WO_GPS': '1'
    })

    px4_sitl = ExecuteProcess(
        cmd=[px4_bin, px4_etc, '-s', 'etc/init.d-posix/rcS', '-i', '0', '-d'],
        cwd=px4_build_path,
        env=px4_env,
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        dds_agent,
        px4_sitl,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_models_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [f"-r ", world_path], 'gz_version': '8'}.items(),
        )
    ])