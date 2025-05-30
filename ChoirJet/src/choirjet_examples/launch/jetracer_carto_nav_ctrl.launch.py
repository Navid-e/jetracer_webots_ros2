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

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from nav2_common.launch import RewrittenYaml

# Flags 
node_frequency = 10 # [Hz]
# number of agents
N = 1
use_sim_time = LaunchConfiguration('use_sim_time', default = True)
package_dir = get_package_share_directory('choirjet_examples')
robot_description_imu = [''] * N

for j in range(N):
    # Here is how to connect the URDF to Webots parameters
    robot_description_imu[j] =  pathlib.Path(os.path.join(package_dir, 'jetracer_{}.urdf'.format(j))).read_text()

def get_jetracer_driver(agent_id):

    
    # Here is how to define the the robot for the 'webots_ros2_driver'
         
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
            {'robot_description': robot_description_imu[agent_id]},
        ]
    )

    return jetracer_driver

def get_jetracer_state_publisher(agent_id):

    # Here is how to define the the robot for the 'robot_state_publisher'

    jetracer_st_pb = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={
                'WEBOTS_ROBOT_NAME':'agent_{}'.format(agent_id),
                },
            namespace='agent_{}'.format(agent_id),
            output='screen',
            parameters=[{
                'robot_description': robot_description_imu[agent_id],
                }])

    return jetracer_st_pb

def generate_launch_description():



    # initialize launch description
    launch_description = [] 

    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'jetracer_x{}.wbt'.format(N)))     


    launch_description.append(webots)
 
    # add executables for each robot
    for i in range(N):

        # webots exec
        launch_description.append(get_jetracer_driver(i))
        launch_description.append(get_jetracer_state_publisher(i))

    ##### Cartographer


    choirjet_examples_prefix = get_package_share_directory('choirjet_examples')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  choirjet_examples_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='my_cartographer.lua')
    
    launch_description.append(DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'))
    launch_description.append(DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'))

    
    launch_description.append(Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            node_name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]))
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.1')


    launch_description.append(DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'))

    launch_description.append(DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'))



    launch_description.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items())
        )
    
    ##### Customized Nodes

    launch_description.append(Node(
            package='choirjet_examples',
            node_executable='path_publisher_node',
            output='screen',            
        ))  
    launch_description.append(Node(
            package='choirjet_examples',
            node_executable='lidar_converter_node',
            output='screen',            
        ))
    launch_description.append(Node(
        package='choirjet_examples',
        node_executable='odom_publisher_node',
        output='screen',            
    ))

    launch_description.append(Node(
            package='choirjet_examples',
            node_executable='ackermann_driver',
            output='screen',            
        )) 
    launch_description.append(Node(
            package='choirjet_examples',
            node_executable='goal_pose_gen',
            output='screen',            
        ))        




    ################# Customized way of launching the NAV2 package (node by node)

    bringup_dir = get_package_share_directory('choirjet_examples')

    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']


    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)


    launch_description.append(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    launch_description.append(DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'))


    launch_description.append(DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'))

    launch_description.append(DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'jetracer_nav.yaml'),
            description='Full path to the ROS2 parameters file to use'))

    launch_description.append(DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'))

    launch_description.append(DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'))

    launch_description.append(Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings))

    launch_description.append(Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings))

    launch_description.append(Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings))

    launch_description.append(Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings))

    launch_description.append(Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings))

    launch_description.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]))



    ############### Launching and config of RViz2

    rviz_config_dir = os.path.join(get_package_share_directory('choirjet_examples'),
                                'rviz', 'my_cartographer.rviz')
    
    launch_description.append(Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'))
    
    return LaunchDescription(launch_description)
    
