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
        xacro.process_doc(doc, mappings={'robot_name': robot_name})
        params = {'robot_description': doc.toxml()}

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        )

        return robot_state_publisher
    
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


    def robot_spawner(entity_name, x, y, z, roll, pitch, yaw):
        position = [x, y, z]
        orientation = [roll, pitch, yaw]
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
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
                '/robot_description',
            ]
        )
        return spawn_robot

    first_robot = 'rick'
    second_robot = 'morty'

    # Robot descriptions, each has a different namespace
    robot_state_publisher_1 = create_robot_state_publisher(first_robot)
    robot_state_publisher_2 = create_robot_state_publisher(second_robot)

    spawner_1 = robot_spawner(first_robot, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0)
    spawner_2 = robot_spawner(second_robot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


    # LAUNCH GAZEBO ON AN EMPTY WORLD
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('barista_robot_description')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "barista_robot_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
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
                os.path.join(pkg_box_bot_gazebo, "worlds", "empty_world.world"),
                "",
            ],
            description="SDF world file",
        )
    )
    ld.add_action(gazebo)
    ld.add_action(TimerAction(period=10.0, actions=[spawner_2]))
    ld.add_action(TimerAction(period=10.0, actions=[spawner_1]))
    ld.add_action(TimerAction(period=12.0, actions=[robot_state_publisher_1]))
    ld.add_action(TimerAction(period=12.0, actions=[robot_state_publisher_2]))
    ld.add_action(TimerAction(period=14.0, actions=[rviz_node]))
    return ld    # Robot Spawner
