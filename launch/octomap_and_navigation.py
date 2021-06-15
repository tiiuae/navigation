#!/usr/bin/env python

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "fog_core"
    pkg_share_path = get_package_share_directory(package_name=pkg_name)
    
    navigation_path = get_package_share_directory(package_name="navigation")
    octomap_server_path = get_package_share_directory(package_name="octomap_server2")

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    namespace=DRONE_DEVICE_ID

    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_octomap_and_navigation',
        package='rclcpp_components',
        executable='component_container_mt',
        # executable='component_container',
        composable_node_descriptions=[
            # navigation 
            ComposableNode(
                namespace=namespace,
                name='navigation',
                package='navigation',
                plugin='navigation::Navigation',
                
                parameters=[
                    navigation_path + '/config/navigation.yaml',
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("~/octomap_in", "/" + DRONE_DEVICE_ID + "/octomap_server/octomap_full"),
                    ("~/odometry_in", "/" + DRONE_DEVICE_ID + "/control_interface/local_odom"),
                    ("~/waypoints_out", "/" + DRONE_DEVICE_ID + "/control_interface/waypoints"),
                    ("~/goto_in", "~/goto_waypoints"),
                    ("~/goto_trigger_in", "~/goto_start"),
                    ("~/set_path_in", "~/set_path"),
                ],

            ),
            # octomap_server
            ComposableNode(
                namespace=namespace,
                name='octomap_server',
                package='octomap_server2',
                plugin='octomap_server::OctomapServer',
                remappings=[
                    # subscribers
                    ('laser_scan_in', 'rplidar/scan'),
                    # publishers
                    ('octomap_binary_out', '~/octomap_binary'),
                    ('octomap_full_out', '~/octomap_full'),
                    ('occupied_cells_vis_array_out', '~/occupied_cells_vis_array'),
                    ('free_cells_vis_array_out', '~/free_cells_vis_array_out'),
                    ('octomap_point_cloud_centers_out', '~/octomap_point_cloud_centers'),
                    ('octomap_free_centers_out', '~/octomap_free_centers_out'),
                    ('projected_map_out', '~/projected_map'),
                    # service servers
                    ('octomap_binary', '~/octomap_binary'),
                    ('octomap_full', '~/octomap_full'),
                    ('clear_bbx', '~/clear_bbx'),
                    ('reset', '~/reset'),
                ],
                parameters=[
                    octomap_server_path + '/config/params.yaml',
                    {"frame_id": "world"},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))
    return ld
