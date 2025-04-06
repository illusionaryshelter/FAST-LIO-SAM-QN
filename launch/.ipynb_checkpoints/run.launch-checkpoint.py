from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os
import yaml

def generate_launch_description():

    fast_lio_dir = get_package_share_directory('fast_lio_sam_qn')
    config_path = os.path.join(fast_lio_dir, 'config', 'config.yaml')  
    
    with open(config_path, 'r') as file:
        configParams = yaml.safe_load(file)['fast_lio_sam_qn_node']['ros__parameters']   
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description=''
        ),
        DeclareLaunchArgument(
            'lidar',
            default_value='mid360',
            description='LiDAR type: ouster, velodyne, mid360, etc.See fast_lio launch folder'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description=''
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/cloud_registered',
            description=''
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_sam',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('fast_lio_sam_qn'),
                'rviz',
                'sam_rviz.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('rviz')),
            prefix='nice',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_lio',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('fast_lio_sam_qn'),
                'rviz',
                'lio_rviz.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('rviz')),
            prefix='nice',
            output='screen'
        ),

        Node(
            package='fast_lio_sam_qn',
            executable='fast_lio_sam_qn_node',
            name='sam_qn_node',
            output='screen',
            parameters=[
                configParams
            ],
            remappings=[
                ('/Odometry', LaunchConfiguration('odom_topic')),
                ('/cloud_registered', LaunchConfiguration('lidar_topic'))
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'ouster'),
            launch_arguments={'rviz': 'false',
                              'config_file': 'ouster64.yaml'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'velodyne'),
            launch_arguments={'rviz': 'false',
                              'config_file': 'velodyne.yaml'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'avia'),
            launch_arguments={'rviz': 'false',
                              'config_file': 'avia.yaml'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio_localization_sc_qn'),
                    'launch',
                    'kitti.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'kitti'),
            launch_arguments={'rviz': 'false'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'horizon'),
            launch_arguments={'rviz': 'false',
                              'config_file': 'horizon.yaml'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('lidar', 'mid360'),
            launch_arguments={'rviz': 'false',
                              'config_file': 'mid360.yaml'}.items()
        ),
    ])
