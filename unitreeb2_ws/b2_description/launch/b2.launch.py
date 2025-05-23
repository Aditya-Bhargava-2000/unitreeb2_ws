import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    b2_description_path = os.path.join(get_package_share_directory('b2_description'))

    xacro_file = os.path.join(b2_description_path,'urdf','b2_description_effort.xacro.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    controller_config_file = os.path.join(b2_description_path, 'config', 'b2_control.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[robot_description, controller_config_file , {'mujoco_model_path':os.path.join(b2_description_path, 'model', 'b2_with_scene.xml')}]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])