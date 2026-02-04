import os
import re
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Base paths
    kortex_pkg_path = get_package_share_directory('kortex_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    xacro_file = os.path.join(kortex_pkg_path, 'robots', 'gen3_lite.urdf')
    original_controllers_file = os.path.join(kortex_pkg_path, 'arms', 'gen3_lite', '6dof', 'config', 'ros2_controllers.yaml')

    # 2. Rewrite YAML (Keep unchanged)
    def rewrite_yaml(original_path, namespace):
        with open(original_path, 'r') as f:
            content = f.read()
        
        nodes_to_rename = ['controller_manager', 'joint_state_broadcaster', 
                           'joint_trajectory_controller', 'gen3_lite_2f_gripper_controller', 'twist_controller']
        for node in nodes_to_rename:
            pattern = re.compile(f'^{node}:', re.MULTILINE)
            content = re.sub(pattern, f'/{namespace}/{node}:', content)

        prefix = f"{namespace}_"
        for i in range(1, 7):
            content = content.replace(f'- joint_{i}', f'- {prefix}joint_{i}')
        
        # Gripper joint name replacement
        content = content.replace('right_finger_bottom_joint', f'{prefix}right_finger_bottom_joint')

        # Inject loose parameters
        extra_params = f"""
            update_rate: 50
            action_monitor_rate: 20.0
            allow_partial_joints_goal: true
        """
        content = re.sub(r'(joint_trajectory_controller:.*?ros__parameters:)', rf'\1{extra_params}', content, flags=re.DOTALL)
        content = re.sub(r'(gen3_lite_2f_gripper_controller:.*?ros__parameters:)', rf'\1{extra_params}', content, flags=re.DOTALL)

        temp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        temp_file.write(content)
        temp_file.close()
        return temp_file.name

    # ==========================================================
    # 3. URDF Processing (Core of physical position correction)
    # ==========================================================
    def get_patched_robot_description(robot_name, robot_ip, x_pos, y_pos, z_pos, yaw_rot, namespaced_controllers_file):
        prefix = f"{robot_name}_"

        doc = xacro.process_file(xacro_file, mappings={
            'name': robot_name, 'arm': 'gen3_lite', 'dof': '6', 'vision': 'false',
            'sim_gazebo': 'false', 'gripper': 'gen3_lite_2f', 'prefix': prefix, 'robot_ip': robot_ip
        })
        xml = doc.toxml()

        # A. Brute-force renaming
        target_words = [
            "base_link", "base_joint", 
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
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
            xml = xml.replace(f'"{word}"', f'"{prefix}{word}"')

        # # B. Replace hardware interface
        # xml = xml.replace('kortex_driver/KortexMultiInterfaceHardware', 'gz_ros2_control/GazeboSimSystem')
        # xml = xml.replace('kortex_driver/KortexGenericSystem', 'gz_ros2_control/GazeboSimSystem')

        # # C. ▼▼▼ Core Physics Fix: Modify base_joint coordinates instead of deleting it ▼▼▼
        # Find the base_joint connecting to world, change its xyz/rpy to the passed parameters
        # This way Gazebo will "nail" it to this position, preventing it from falling due to gravity
        joint_name = f"{prefix}base_joint"
        new_origin = f'<origin xyz="{x_pos} {y_pos} {z_pos}" rpy="0 0 {yaw_rot}"/>'
        
        # Use regex to find the joint block and replace the origin inside
        # Logic here: Find the base_joint definition block, keep head and tail, only replace the origin in the middle
        pattern = rf'(<joint name="{joint_name}" type="fixed">.*?)(<origin .*?/>)(.*?</joint>)'
        xml = re.sub(pattern, rf'\1{new_origin}\3', xml, flags=re.DOTALL)

        # # D. Inject Mimic plugins (Fix fingers)
        # mimic_plugins = f"""
        # <gazebo>
        #   <plugin filename="gz-sim-mimic-joint-plugin" name="gz::sim::systems::MimicJoint">
        #     <joint>{prefix}right_finger_bottom_joint</joint>
        #     <mimicJoint>{prefix}left_finger_bottom_joint</mimicJoint>
        #     <multiplier>1.0</multiplier>
        #     <offset>0.0</offset>
        #   </plugin>
        #   <plugin filename="gz-sim-mimic-joint-plugin" name="gz::sim::systems::MimicJoint">
        #     <joint>{prefix}right_finger_bottom_joint</joint>
        #     <mimicJoint>{prefix}right_finger_tip_joint</mimicJoint>
        #     <multiplier>-0.676</multiplier>
        #     <offset>0.149</offset>
        #   </plugin>
        #   <plugin filename="gz-sim-mimic-joint-plugin" name="gz::sim::systems::MimicJoint">
        #     <joint>{prefix}right_finger_bottom_joint</joint>
        #     <mimicJoint>{prefix}left_finger_tip_joint</mimicJoint>
        #     <multiplier>-0.676</multiplier>
        #     <offset>0.149</offset>
        #   </plugin>
        # </gazebo>
        # """
        # xml = xml.replace('</robot>', mimic_plugins + '</robot>')

        # # E. Inject plugins (With PID gains)
        # new_plugin_xml = f"""
        # <gazebo>
        #   <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        #     <parameters>{namespaced_controllers_file}</parameters>
        #     <ros>
        #       <namespace>/{robot_name}</namespace>
        #       <parameter name="use_sim_time" value="true"/>
        #     </ros>
        #     <parameter name="position_proportional_gain" value="50.0" />
        #   </plugin>
        # </gazebo>
        # """
        # return xml.replace('</robot>', new_plugin_xml + '</robot>')

        return xml
    # 4. Create nodes
    def create_arm_nodes(robot_name, robot_ip, x, y, z, yaw, delay_sec):
        # ▼▼▼ Key: Pass coordinates to the processing function, write inside URDF ▼▼▼
        namespaced_controllers_file = rewrite_yaml(original_controllers_file, robot_name)
        patched_desc = get_patched_robot_description(robot_name, robot_ip, x, y, z, yaw, namespaced_controllers_file)
        nodes = []

        # RSP will publish correct TF based on modified URDF (e.g., world -> base = 1.0)
        state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot_name, output='screen',
            parameters=[{'robot_description': patched_desc}],
        )

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_name,
            parameters=[namespaced_controllers_file],
            remappings=[
                ("~/robot_description", f"/{robot_name}/robot_description"),
            ],
            output="both",
        )
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_name,
            arguments=[
                "joint_state_broadcaster",
                "-c",
                f"/{robot_name}/controller_manager",
            ],
        )
        robot_traj_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_name,
            arguments=['joint_trajectory_controller', "-c", f"/{robot_name}/controller_manager"],
        )

        robot_hand_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_name,
            arguments=["gen3_lite_2f_gripper_controller", "-c", f"/{robot_name}/controller_manager"],
        )

        # only start the fault controller if we are using hardware
        fault_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_name,
            arguments=["fault_controller", "-c", f"/{robot_name}/controller_manager"],
        )
        nodes = [
            state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            robot_traj_controller_spawner,
            robot_hand_controller_spawner,
            fault_controller_spawner,
        ]

        # Spawn (Note: xyz must be set to 0! Because offsets are already written inside URDF)
        return TimerAction(period=delay_sec, actions=nodes)

 
    # ❌ Deleted Static TF
    # Because URDF now has parent="world", RSP will automatically publish correct world->base transform
    # We don't need to manually publish TF anymore, otherwise they will conflict

    return LaunchDescription([
        # ▼▼▼ Adjust positions freely here ▼▼▼
        # Params: (name, x, y, z, yaw, delay)
        #TODO robot ip
        create_arm_nodes('left_arm', "192.168.10.0", 0.0, 0.0, 0.3, 0.0, 4.0),
        # create_arm_nodes('right_arm', 1.25, 0.0, 0.3, 3.14159, 10.0),
         create_arm_nodes('right_arm', "192.168.10.0", 0.84, 0.0, 0.3, 3.14159, 10.0),
    ])