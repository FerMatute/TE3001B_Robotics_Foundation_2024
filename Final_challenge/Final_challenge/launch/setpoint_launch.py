import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('Final_challenge'),
        'config',
        'params.yaml'
    )

    setpoint_node = Node(
        package='Final_challenge',
        name='Setpoint',
        executable='Setpoint',
        output='screen',
        parameters=[config],
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )

    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
    )

    ld.add_action(setpoint_node)
    ld.add_action(rqt_graph_node)
    ld.add_action(rqt_plot_node)

    return ld