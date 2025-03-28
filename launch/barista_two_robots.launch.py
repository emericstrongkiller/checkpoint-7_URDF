import os
import random

from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

import xacro

# this is the function launch  system will look for
def generate_launch_description():

    def create_robot_state_publisher(robot_name):
        robot_model_path = os.path.join(
            get_package_share_directory('barista_robot_description'))

        xacro_file = os.path.join(robot_model_path, 'xacro', 'barista_very_simple_main.xacro')

        # convert XACRO file into URDF
        doc = xacro.parse(open(xacro_file))
        #xacro.process_doc(doc)
        #xacro.process_doc(doc, mappings={'robot_name': robot_name})
        xacro.process_doc(doc, mappings={'robot_name_arg': robot_name})
        params = {'robot_description': doc.toxml()}
        #params = {'robot_description': doc.toxml(),
        #  'frame_prefix': robot_name + '/'}

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params],
            remappings=[
                ('/robot_description', f'/{robot_name}/robot_description'),
                ('/joint_states', f'/{robot_name}/joint_states') # ADD THIS LINE
            ]
        )

        return robot_state_publisher

    def generate_origin(robot_name):
        # Add to your launch file
        static_tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_origin_to_odom', # Unique name
            arguments=['0', '0', '0', '0', '0', '0', 'origin', f'{robot_name}/odom']
        )
        return static_tf_publisher
    
    # RVIZ Configuration
    package_description = "barista_robot_description"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_conf.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    def robot_spawner(entity_name, x, y, z, roll, pitch, yaw, robot_name):
        position = [x, y, z]
        orientation = [roll, pitch, yaw]
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'{robot_name}_spawner', # Unique name
            output='screen',
            arguments=[
                '-entity',
                entity_name,
                '-x',
                str(position[0]),
                '-y',
                str(position[1]),
                '-z',
                str(position[2]),
                '-R',
                str(orientation[0]),
                '-P',
                str(orientation[1]),
                '-Y',
                str(orientation[2]),
                '-topic',
                f'/{robot_name}/robot_description',
            ]
        )
        return spawn_robot

    first_robot = 'morty'
    second_robot = 'rick' #(morty)

    # Robot descriptions, each has a different namespace
    robot_state_publisher_1 = create_robot_state_publisher(first_robot)
    robot_state_publisher_2 = create_robot_state_publisher(second_robot)

    spawner_1 = robot_spawner(first_robot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, first_robot)
    spawner_2 = robot_spawner(second_robot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, second_robot)

    #origin generator for both robots
    second_robot_origin_1 = generate_origin(first_robot)
    second_robot_origin_2 = generate_origin(second_robot)

    # LAUNCH GAZEBO ON AN EMPTY WORLD
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    barista_description = get_package_share_directory('barista_robot_description')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "barista_robot_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in barista description package
    gazebo_models_path = os.path.join(barista_description, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    
    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    


    # create and return launch description object with delay between each launch
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value=[
                os.path.join(barista_description, "worlds", "empty_world.world"),
                "",
            ],
            description="SDF world file",
        )
    )
    ld.add_action(gazebo)
    ld.add_action(TimerAction(period=10.0, actions=[spawner_1]))
    ld.add_action(TimerAction(period=12.0, actions=[spawner_2]))
    ld.add_action(TimerAction(period=14.0, actions=[robot_state_publisher_1]))
    ld.add_action(TimerAction(period=16.0, actions=[robot_state_publisher_2]))
    ld.add_action(TimerAction(period=18.0, actions=[second_robot_origin_1]))
    ld.add_action(TimerAction(period=20.0, actions=[second_robot_origin_2]))
    ld.add_action(TimerAction(period=22.0, actions=[rviz_node]))
    return ld    # Robot Spawner
