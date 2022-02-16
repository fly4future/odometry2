import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import sys

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "odometry2"
    pkg_share_path = get_package_share_directory(pkg_name)
 
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("world_frame", default_value="world"))

    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_odometry',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='odometry2::Odometry2',
                namespace=namespace,
                name='odometry',
                parameters=[
                    pkg_share_path + '/config/param.yaml',
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                    {"world_frame": launch.substitutions.LaunchConfiguration("world_frame")},
                ],
                remappings=[
                    # publishers
                    ("~/local_odom_out", "~/local_odom"),
                    ("~/local_hector_out", "~/local_hector"),
                    ("~/hector_odometry_out", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_visual_odometry/in"),
                    ("~/pixhawk_hector_out", "~/pixhawk_hector"),
                    ("~/odometry_diagnostics_out", "~/odometry_diagnostics"),
                    ("~/gps_diagnostics_out", "~/gps_diagnostics"),
                    ("~/hector_diagnostics_out", "~/hector_diagnostics"),
                    # subscribers
                    ("~/pixhawk_odom_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_odometry/out"),
                    ("~/timesync_in", "/" + DRONE_DEVICE_ID + "/fmu/timesync/out"),
                    ("~/gps_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_gps_position/out"),
                    ("~/hector_pose_in", "/" + DRONE_DEVICE_ID + "/hector_mapping/slam_out_pose"),
                    ("~/control_interface_diagnostics_in", "/" + DRONE_DEVICE_ID + "/control_interface/diagnostics"),
                    # service_providers
                    ("~/reset_hector_service_in", "~/reset_hector_service"),
                    ("~/change_odometry_source_in", "~/change_odometry_source"),
                    # service_clients
                    ("~/set_px4_param_int", "/" + DRONE_DEVICE_ID + "/control_interface/set_px4_param_int"),
                    ("~/set_px4_param_float", "/" + DRONE_DEVICE_ID + "/control_interface/set_px4_param_float"),
                    ("~/reset_hector_service_out", "/" + DRONE_DEVICE_ID + "/hector_mapping/reset_hector"),
                    ("~/land_service_out", "/" + DRONE_DEVICE_ID + "/control_interface/land"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
