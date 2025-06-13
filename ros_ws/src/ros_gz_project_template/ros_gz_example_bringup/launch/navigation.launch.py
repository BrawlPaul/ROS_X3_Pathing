from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    nav2_config = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_nav2),
            launch_arguments={
                'use_sim_time': 'True',
                'map': '/home/benjamin/Documents/seniorProject/maps/1_5Map.yaml',
                'params_file': nav2_config
            }.items()
        )
    pkg_nav2_rviz = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_nav2_rviz)
    )
    
    return LaunchDescription([
        nav2,
        nav2_rviz
    ])