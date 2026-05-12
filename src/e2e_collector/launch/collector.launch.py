import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    
    # 1. Launch TB4 Simulator (Gazebo Harmonic + Nav2)
    # The 'nav2:=true' flag starts Nav2's own lifecycle manager
    pkg_e2e_collector = get_package_share_directory('e2e_collector')
    custom_nav2_params = os.path.join(pkg_e2e_collector, 'config', 'nav2_params.yaml')
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz, 'launch', 'turtlebot4_gz.launch.py')
        ),
        launch_arguments={'nav2': 'true', 'slam': 'true', 'world': 'maze', 'rviz': 'true', 'params_file': custom_nav2_params,}.items()
    )

    # 2. Your Data Collection Lifecycle Node
    collector_node = LifecycleNode(
        package='e2e_collector',
        executable='data_logger',
        name='e2e_data_collector',
        namespace='',
        output='screen'
    )

    frontier_node = Node(
        package='e2e_collector',
        executable='frontier_navigator',
        name='frontier_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Add your node to Nav2's Lifecycle Manager
    # This ensures your node follows the same startup/shutdown sequence as Nav2
    lc_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='e2e_lc_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['slam_toolbox', 
                                    'e2e_data_collector', 
                                    'frontier_navigator']}]
    )

    # keepout_filter_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='filter_mask_server',
    #     output='screen',
    #     parameters=[{
    #         'yaml_filename': '/ros2_ws/src/e2e_collector/maps/keepout.yaml',
    #         'use_sim_time': True
    #     }]
    # )

    # costmap_filter_info_server = Node(
    #     package='nav2_map_server',
    #     executable='costmap_filter_info_server',
    #     name='costmap_filter_info_server',
    #     output='screen',
    #     parameters=[{
    #         'type': 0, # 0 for keepout zones
    #         'filter_info_topic': '/costmap_filter_info',
    #         'mask_topic': '/keepout_filter_mask',
    #         'base_variable': 0.0,
    #         'multiplier': 1.0
    #     }]
    # )

    # delayed_collector_and_manager = TimerAction(
    #     period=10.0,
    #     actions=[
    #         collector_node,
    #         lc_manager,
    #         # If you have a separate spawn node for the dock, put it here
    #     ]
    # )

    return LaunchDescription([
        simulator,
        # delayed_collector_and_manager,
        frontier_node,
        collector_node,
        lc_manager,
        # keepout_filter_node,
        # costmap_filter_info_server
    ])