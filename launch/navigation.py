import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.actions import SetEnvironmentVariable
from launch_ros.descriptions import ComposableNode
import os
import sys

def generate_launch_description():

    # ld = launch.LaunchDescription([SetEnvironmentVariable("LD_PRELOAD", "/usr/lib/x86_64-linux-gnu/libasan.so.5")])
    ld = launch.LaunchDescription()

    pkg_name = "navigation"
    pkg_share_path = get_package_share_directory(pkg_name)
    
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])


    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_navigation',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='navigation::Navigation',
                namespace=namespace,
                name='navigation',
                parameters=[
                    pkg_share_path + '/config/navigation.yaml',
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("~/octomap_in", "/" + DRONE_DEVICE_ID + "/octomap_server/octomap/local/full"),
                    ("~/odometry_in", "/" + DRONE_DEVICE_ID + "/odometry/local_odom"),
                    ("~/cmd_pose_in", "/" + DRONE_DEVICE_ID + "/control_interface/cmd_pose"),
                    ("~/hover_in", "~/hover"),
                    ("~/goto_in", "~/goto_waypoints"),
                    ("~/control_diagnostics_in",  "/" + DRONE_DEVICE_ID + "/control_interface/diagnostics"),
                    ("~/bumper_in",  "/" + DRONE_DEVICE_ID + "/bumper/obstacle_sectors"),

                    ("~/local_waypoint_in", "~/local_waypoint"),
                    ("~/local_path_in", "~/local_path"),
                    ("~/gps_waypoint_in", "~/gps_waypoint"),
                    ("~/gps_path_in", "~/gps_path"),
                    ("~/diagnostics_out", "~/diagnostics"),

                    ("~/status_out", "~/status_out"), #needs to be named as status_out due to mission-engine plugin
                    ("~/future_trajectory_out", "~/future_trajectory"),

                    ("~/binary_tree_markers_out", "~/visualization/binary_tree_markers"),
                    ("~/expansion_markers_out", "~/visualization/expansion_markers"),
                    ("~/goal_markers_out", "~/visualization/goal_markers"),
                    ("~/path_markers_out", "~/visualization/path_markers"),

                    ("~/local_path_out", "/" + DRONE_DEVICE_ID + "/control_interface/local_path"),
                    ("~/waypoint_to_local_out", "/" + DRONE_DEVICE_ID + "/control_interface/waypoint_to_local"),
                    ("~/path_to_local_out", "/" + DRONE_DEVICE_ID + "/control_interface/path_to_local"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
