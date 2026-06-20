import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_sim = get_package_share_directory('drone_scene')
    pkg_node = get_package_share_directory('drone_control')


    # Launch Simulation (Gazebo, RViz, Spawn robot, Bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'coverage_sim.launch.py')
        )
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_node',
        arguments=['/camera/image_debug'], 
        output='screen'
    )

    coverage_node = Node(
        package="drone_control",
        executable="coverage_node",
        name="coverage_node",
        output="screen",
    )

    coverage_docking_node = Node(
        package="drone_control",
        executable="coverage_docking_node",
        name="coverage_docking_node",
        output="screen"
    )

    return LaunchDescription([
        sim_launch,
        # TimerAction(period=7.0, actions=[coverage_node]),
        TimerAction(period=7.0, actions=[coverage_docking_node]),
        # rqt_image_view_node,
    ])
