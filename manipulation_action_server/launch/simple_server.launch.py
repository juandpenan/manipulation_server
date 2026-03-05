# Copyright (c) 2024 Intelligent Robotics Lab - Gentlebots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix
from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from tiago_description.launch_arguments import TiagoArgs
from launch.substitutions import LaunchConfiguration
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor
    wrist_model: DeclareLaunchArgument = TiagoArgs.wrist_model
    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model

    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    launch_description.add_action(OpaqueFunction(function=start_simple_manipulation_server))
    return


def start_simple_manipulation_server(context, *args, **kwargs):
    base_type = read_launch_argument("base_type", context)
    arm_type = read_launch_argument("arm_type", context)
    end_effector = read_launch_argument("end_effector", context)
    ft_sensor = read_launch_argument("ft_sensor", context)

    hw_suffix = get_tiago_hw_suffix(
        arm=arm_type,
        end_effector=end_effector,
    )

    srdf_file_path = os.path.join(
        get_package_share_directory("tiago_moveit_config"),
        "config", "srdf",
        "tiago.srdf.xacro",
    )

    srdf_input_args = {
        "arm_type": arm_type,
        "end_effector": end_effector,
        "ft_sensor": ft_sensor,
        "base_type": base_type,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = f"config/controllers/controllers{hw_suffix}.yaml"

    robot_description_kinematics = "config/kinematics_kdl.yaml"
    joint_limits = "config/joint_limits.yaml"
    pilz_cartesian_limits = "config/pilz_cartesian_limits.yaml"

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Collision padding: adds a safety buffer around robot links during planning.
    # default_robot_padding applies globally; per-link overrides go in link_padding.
    collision_padding = {
        "robot_description_planning": {
            "default_robot_padding": 0.01,   # 1 cm global padding on all links
            "default_robot_scale": 1.0,
            # Uncomment and tune per-link padding if needed:
            # "link_padding": {
            #     "arm_1_link": 0.02,
            #     "arm_2_link": 0.02,
            #     "gripper_link": 0.01,
            # },
        },
        # Execution timeout: MoveGroupInterface checks these to decide if a controller
        # is taking too long. Increase margin to avoid CONTROL_FAILED (-4) on long motions.
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 2.0,   # allow 2x the expected duration
            "allowed_goal_duration_margin": 40.0,         # plus 30s absolute margin
            "execution_duration_monitoring": True,
        },
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder("tiago")
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=robot_description_kinematics)
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .joint_limits(file_path=joint_limits)
        .planning_pipelines(
            pipelines=["ompl"], default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits)
        .to_moveit_configs()
    )

    simple_server_configuration = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "planning_group": "arm",
    }

    simple_server_params = [
        moveit_config.to_dict(),
        simple_server_configuration,
        collision_padding,
    ]

    # Start the simple manipulation server node
    simple_server_node = Node(
        package="manipulation_action_server",
        executable="simple_manipulation_server",
        output="screen",
        emulate_tty=True,
        parameters=simple_server_params,
    )

    return [simple_server_node]
