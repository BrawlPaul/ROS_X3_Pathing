# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_kiss_icp = get_package_share_directory('kiss_icp')
    # Point to additional launch file
    pkg_nav2 = os.path.join(get_package_share_directory('ros_gz_example_bringup'), 'launch', 'navigation.launch.py')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'x3', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'x3.sdf '
            #'--render-engine ogre'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'x3.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    # Start up robot localization
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[os.path.join(pkg_project_bringup, 'config', 'x3_ekf.yaml'), 
    {'use_sim_time': True}])

    
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_x3_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    altimeter_publisher = Node(
        package='x3_pathing',
        executable='altimeter_publisher',
        output='screen'
    )

    altitude_controller = Node(
        package='x3_pathing',
        executable='altitude_controller',
        output='screen'
    )

    kiss_icp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_kiss_icp, 'launch', 'odometry.launch.py')),
            launch_arguments={
                'publish_odom_tf': 'False',
                'base_frame': 'x3/base_link',
                'lidar_odom_frame': 'x3/odom',
                'invert_odom_tf': 'False',
                'topic': '/x3/scan/points',
                'use_sim_time': 'true'

            }.items()
    )   


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_nav2)
    )



    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        rviz,
        altimeter_publisher,
        altitude_controller,
        kiss_icp,
        # Static transform for IMU0
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "x3/base_link", "x3/base_link/imu_sensor0"],
            name="static_tf_imu1"
        ),
        
        # Static transform for IMU1
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "x3/base_link", "x3/base_link/imu_sensor1"],
            name="static_tf_imu2"
        ),
        
        # Static transform for altimeter
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "x3/base_link", "x3/base_link/altimeter"],
            name="static_tf_altimeter"
        ),

        # Static transform for LIDAR
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "x3/base_link", "x3/base_link/lidar"],
            name="static_tf_lidar"
        ),

        start_robot_localization_cmd,
        # nav2,
    ])
