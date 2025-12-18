import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description(): 

    ld = LaunchDescription()

    mappings = {
        'arm': 'tiago-arm',
        'camera_model': 'orbbec-astra',
        'end_effector': 'pal-gripper',
        'ft_sensor': 'schunk-ft',
        'laser_model': 'sick-571',
        'wrist_model': 'wrist-2010'
    }

    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description(file_path=os.path.join(
            get_package_share_directory('tiago_description'),
            'robots', 'tiago.urdf.xacro'), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'srdf', 'tiago.srdf.xacro'))
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'kinematics_kdl.yaml'))
        .trajectory_execution(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'controllers', 'controllers_pal-gripper.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor({
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        })
        .pilz_cartesian_limits(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    simple_server_cmd = Node(
        package='manipulation_action_server',
        executable='simple_manipulation_server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            {'planning_group': 'arm_torso'}
        ]
    )

    ld.add_action(simple_server_cmd)
    return ld
