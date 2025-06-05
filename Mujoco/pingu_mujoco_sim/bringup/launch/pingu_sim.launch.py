import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Set arguments TODO: make this a LaunchConfiguration
    prefix = ''
    floating_joint = 'true'
    ros2_control = 'true'
    hw_plugin = 'mujoco'
    left_arm = 'true'
    right_arm = 'true'    

    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('pingu_mujoco_sim'),)

    xacro_file = os.path.join(get_package_share_directory('pingu_description'),
                              'urdf',
                              'pingu.urdf.xacro')
    # load xacro
    doc = xacro.process_file(xacro_file, 
        mappings={'prefix': prefix, 
                  'floating_joint': floating_joint,
                  'ros2_control': ros2_control,
                  'hw_plugin': hw_plugin,
                  'left_arm': left_arm,
                  'right_arm': right_arm})
    robot_description = {'robot_description': doc.toxml()}

    controller_config_file = os.path.join(get_package_share_directory('pingu_ros2_control'), 'config', 'pingu_controllers.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'pingu.xml')}
        ]
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

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'rw_effort_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])