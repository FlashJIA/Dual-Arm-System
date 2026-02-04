import os
import xacro
import yaml
import re
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    nodes = []
    
    kortex_pkg = get_package_share_directory('kortex_description')
    moveit_config_pkg = get_package_share_directory('kinova_gen3_lite_moveit_config')
    
    urdf_path = os.path.join(kortex_pkg, 'robots', 'gen3_lite.urdf')
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'gen3_lite.srdf')
    kinematics_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    ompl_path = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
    joint_limits_path = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')

    rviz_params_dict = {'use_sim_time': True}

    def prepare_robot_config(robot_name, x_pos, yaw_rot):
        local_nodes = [] 
        prefix = f"{robot_name}_"
        
        doc = xacro.process_file(urdf_path, mappings={
            'name': robot_name, 'arm': 'gen3_lite', 'dof': '6', 'vision': 'false', 
            'sim_gazebo': 'false', 'gripper': 'gen3_lite_2f', 'prefix': prefix
        })
        urdf_xml = doc.toxml()

        target_words = [
            "base_link", "base_joint", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
            "link_1", "link_2", "link_3", "link_4", "link_5", "link_6",
            "end_effector_link", "tool_frame", "end_effector",
            "shoulder_link", "arm_link", "forearm_link", 
            "lower_wrist_link", "upper_wrist_link", "dummy_link",
            "gripper_base_link", "right_finger_prox_link", "right_finger_dist_link",
            "left_finger_prox_link", "left_finger_dist_link",
            "right_finger_bottom_joint", "right_finger_tip_joint",
            "left_finger_bottom_joint", "left_finger_tip_joint"
        ]
        for word in target_words:
            urdf_xml = urdf_xml.replace(f'"{word}"', f'"{prefix}{word}"')

        urdf_xml = re.sub(r'<limit([^>]*)>', r'<limit acceleration="10.0"\1>', urdf_xml)

        joint_name = f"{prefix}base_joint"
        new_origin = f'<origin xyz="{x_pos} 0 0.3" rpy="0 0 {yaw_rot}"/>'
        pattern = rf'(<joint name="{joint_name}" type="fixed">.*?)(<origin .*?/>)(.*?</joint>)'
        urdf_xml = re.sub(pattern, rf'\1{new_origin}\3', urdf_xml, flags=re.DOTALL)

        robot_description_content = urdf_xml

        with open(srdf_path, 'r') as f: srdf_content = f.read()
        patch_srdf = srdf_content
        for word in target_words:
            patch_srdf = patch_srdf.replace(f'"{word}"', f'"{prefix}{word}"')
        
        patch_srdf = re.sub(r'<virtual_joint .*?/>', '', patch_srdf, flags=re.DOTALL)
        
        with open(kinematics_path, 'r') as f: kinematics_yaml = yaml.safe_load(f)
        with open(ompl_path, 'r') as f: ompl_yaml = yaml.safe_load(f)
        with open(joint_limits_path, 'r') as f: original_limits = yaml.safe_load(f)

        ompl_config = {'planning_pipelines': ['ompl'], 'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ['default_planner_request_adapters/AddTimeOptimalParameterization', 
                                 'default_planner_request_adapters/ResolveConstraintFrames',
                                 'default_planner_request_adapters/ValidateWorkspaceBounds',
                                 'default_planner_request_adapters/CheckStartStateBounds',
                                 'default_planner_request_adapters/CheckStartStateCollision'],
            'start_state_max_bounds_error': 0.1}}
        if ompl_yaml: ompl_config['ompl'].update(ompl_yaml)

        patched_limits = {'joint_limits': {}}
        if 'joint_limits' in original_limits:
            for joint, params in original_limits['joint_limits'].items():
                patched_limits['joint_limits'][f"{prefix}{joint}"] = params
                patched_limits['joint_limits'][f"{prefix}{joint}"]['has_acceleration_limits'] = True
                patched_limits['joint_limits'][f"{prefix}{joint}"]['max_acceleration'] = 5.0

        # --- Controller Configuration ---
        controllers_yaml = {
            'moveit_simple_controller_manager': {
                'controller_names': ['joint_trajectory_controller', 'gen3_lite_2f_gripper_controller'],
                'joint_trajectory_controller': {
                    'type': 'FollowJointTrajectory',
                    'action_ns': 'follow_joint_trajectory',
                    'default': True,
                    'joints': [f'{prefix}joint_{i}' for i in range(1, 7)]
                },
                'gen3_lite_2f_gripper_controller': {
                    'type': 'GripperCommand',
                    'action_ns': 'gripper_cmd',
                    'default': True,
                    'joints': [f'{prefix}right_finger_bottom_joint']
                }
            },
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        }

        # --- Trajectory Execution Parameters (Critical Fix) ---
        trajectory_execution_params = {
            'moveit_manage_controllers': True,
            'trajectory_execution': {
                'allowed_execution_duration_scaling': 2.0,  # Allow it to be slower
                'allowed_goal_duration_margin': 0.5,
                'allowed_start_tolerance': 0.05,
            }
        }

        move_group = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            namespace=robot_name,
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'robot_description_semantic': patch_srdf},
                {'robot_description_kinematics': kinematics_yaml},
                {'robot_description_planning': patched_limits},
                {'use_sim_time': True},
                ompl_config,
                controllers_yaml,
                trajectory_execution_params, # Add parameters
            ]
        )
        local_nodes.append(move_group)

        rviz_params_dict[f'{robot_name}_description'] = robot_description_content
        rviz_params_dict[f'{robot_name}_description_semantic'] = patch_srdf
        rviz_params_dict[f'{robot_name}_description_kinematics'] = kinematics_yaml

        return local_nodes

    nodes.extend(prepare_robot_config('left_arm', 0.0, 0.0))
    # nodes.extend(prepare_robot_config('right_arm', 1.25, 3.14159))
    nodes.extend(prepare_robot_config('right_arm', 0.84, 3.14159))

    rviz_config_content = {'/rviz2': {'ros__parameters': rviz_params_dict}}
    rviz_params_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(rviz_config_content, rviz_params_file)
    rviz_params_file.close()

    rviz_display_config = "/root/roboticsfinalproject/ros2_ws/src/final_moveit/launch/dual_moveit.rviz"

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_display_config, '--ros-args', '--params-file', rviz_params_file.name],
    )
    nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])