import os
from time import sleep
import numpy as np
import pathlib

import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher


# Flags 
node_frequency = 10 # [Hz]

package_dir = get_package_share_directory('choirjet_examples')
robot_description = pathlib.Path(os.path.join(package_dir, 'jetracer.urdf')).read_text()
def get_jetracer_driver(agent_id):

    

    jetracer_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        # prefix='xterm -title "jetson_driver_{}" -hold -e'.format(agent_id),
        additional_env={
            'WEBOTS_ROBOT_NAME':'agent_{}'.format(agent_id),
            },
        namespace='agent_{}'.format(agent_id),
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return jetracer_driver

def generate_launch_description():

    # number of agents
    N = 2

    # initialize launch description
    launch_description = [] 




    """
    launch_description.append(Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{
                'speed': 1.0,
                'turn': 1.0,
                'repeat_rate': 0.0,
                'topic': '/cmd_vel',
                'msg_type': 'AckermannDrive'
            }]
        ))
    """
    # Launch control Panel
    launch_description.append(Node(
                package='choirjet_examples', 
                executable='choirjet_simple_control_panel',
                output='screen',
                # prefix='xterm -title "control_panel" -hold -e',
                parameters=[{
                    'n_agents': N,
                    }]))

    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'jetracer_x{}.wbt'.format(N)))
        






    launch_description.append(webots)
 
    # add executables for each robot
    for i in range(N):

        # webots exec
        launch_description.append(get_jetracer_driver(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={
                'WEBOTS_ROBOT_NAME':'agent_{}'.format(i),
                },
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                }]))

    # Get the path to the configuration files
    config_directory = get_package_share_directory('choirjet_examples') + '/resource'
    flags_file = config_directory + '/my_flags.lua'
    cartographer_config_file = config_directory + '/my_cartographer.lua'

    launch_description.append(Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[cartographer_config_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        arguments=['--ros-arg', '--params-file',cartographer_config_file,'--params-file',flags_file]
    ))


    return LaunchDescription(launch_description)
    """Node(
    package='cartographer_ros',
    executable='cartographer_node',
    output='screen',
    parameters=[{'home/navid/01_my_packages/Pichierri_05/ChoirJet/src/choirjet_examples/resource/my_cartographer.lua':'value_of_parameter'}
    ],
     )"""
if __name__ == '__main__': 
        generate_launch_description()