#!/usr/bin/env python3
import time
import threading
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from control_msgs.action import GripperCommand

# ==========================================
#           1. Golden tuning parameters
# ==========================================
TOP_DOWN_QUAT = (-0.7071, 0.7071, 0.0, 0.0)
BOARD_CENTER_X = 0.42  # corresponds to spawn.sh X=0.26
SPACING = 0.08

# Height configuration
TEST_WRIST_Z = 0.485   # pick height
TILE_OFFSET = 0.002    # tile thickness offset
PLACE_WRIST_Z = TEST_WRIST_Z + TILE_OFFSET  # placement height (0.487)
CRUISE_Z = TEST_WRIST_Z + 0.10              # cruise height

def make_pose_stamped(x, y, z, frame_id="world") -> PoseStamped:
    """Helper function to generate a PoseStamped with top-down orientation."""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rclpy.time.Time().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.x = TOP_DOWN_QUAT[0]
    p.pose.orientation.y = TOP_DOWN_QUAT[1]
    p.pose.orientation.z = TOP_DOWN_QUAT[2]
    p.pose.orientation.w = TOP_DOWN_QUAT[3]
    return p

# ==========================================
#           2. Arm Controller (copied from dual_arm_tasks.py)
# ==========================================
class ArmController:
    def __init__(self, ros_node, arm_name, namespace, gripper_joint_name, custom_retract=None):
        self.node = ros_node
        self.arm_name = arm_name
        
        # MoveIt2 configuration
        self.moveit = MoveIt2(
            node=self.node,
            joint_names=[f"{arm_name}_joint_{i}" for i in range(1, 7)],
            base_link_name=f"{arm_name}_base_link",
            end_effector_name=f"{arm_name}_end_effector_link",
            group_name="arm",
        )
        self.gripper_client = ActionClient(self.node, GripperCommand, f"{namespace}/gen3_lite_2f_gripper_controller/gripper_cmd")
        
        # Home joint configuration
        self.j_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Retract pose configuration
        if custom_retract:
            self.j_retract = custom_retract
        else:
            # default left-arm retract pose
            self.j_retract = [0.471, 0.332, 2.112, -1.571, -1.292, 2.042]

        # estimated cube size
        self.block_size = 0.04
        self.add_table_collision()

    def add_table_collision(self):
        """Add a collision object for the table."""
        try:
            self.moveit.add_collision_box("table_ground", (2.0, 1.0, 0.29), (0.5, 0.0, 0.145), (0,0,0,1), "world")
        except:
            pass

    def move_to_joints(self, joint_positions):
        """Move arm to a set of joint angles."""
        self.moveit.move_to_configuration(joint_positions=joint_positions)
        self.moveit.wait_until_executed()

    def move_to_pose(self, pose, cartesian=False):
        """Move arm to a Cartesian pose."""
        traj = self.moveit.plan(pose=pose, cartesian=cartesian, max_step=0.005)
        if traj:
            self.moveit.execute(traj)
            self.moveit.wait_until_executed()
            return True
        return False

    # --- Gripper control (with your 5.5 second wait logic) ---
    def _send_gripper(self, pos, wait):
        """Send gripper action goal and optionally wait."""
        if self.gripper_client.wait_for_server(1.0):
            goal = GripperCommand.Goal()
            goal.command.position = pos
            goal.command.max_effort = 100.0
            self.gripper_client.send_goal_async(goal)
            if wait > 0:
                time.sleep(wait)

    def open(self): self._send_gripper(0.45, 5.5)   # open gripper
    def close(self): self._send_gripper(0.70, 5.5)  # close gripper
    
    def retract(self):
        """Bring gripper half-closed and move to retract posture."""
        self._send_gripper(0.40, 0.2)
        self.move_to_joints(self.j_retract)

    # --- Core sequence: Pick -> Cruise -> Place ---
    def execute_move_sequence(self, pick_xyz, place_xyz, obj_id):
        """Full pick-and-place sequence with cruise motion."""
        px, py, _ = pick_xyz  # ignore YAML Z, use TEST_WRIST_Z
        tx, ty, _ = place_xyz
        
        self.node.get_logger().info(f"[{self.arm_name}] Execute: {obj_id}")
        
        # 1. Add collision object for the picked cube
        try:
            self.moveit.add_collision_box(obj_id, (self.block_size,)*3, (px, py, 0.32), (0,0,0,1), "world")
        except:
            pass

        # Pick procedure
        self.open()
        if not self.move_to_pose(make_pose_stamped(px, py, CRUISE_Z)): return False
        if not self.move_to_pose(make_pose_stamped(px, py, TEST_WRIST_Z), True): return False
        
        self.close()
        
        # Attach cube to gripper
        try:
            self.moveit.attach_collision_object(
                obj_id,
                f"{self.arm_name}_end_effector_link",
                [f"{self.arm_name}_right_finger_bottom_link", f"{self.arm_name}_left_finger_bottom_link"]
            )
        except:
            pass

        # 2. Cruise motion: lift then translate
        if not self.move_to_pose(make_pose_stamped(px, py, CRUISE_Z), True): return False
        if not self.move_to_pose(make_pose_stamped(tx, ty, CRUISE_Z), False): return False

        # 3. Place motion
        if not self.move_to_pose(make_pose_stamped(tx, ty, PLACE_WRIST_Z), True): return False
        
        # Detach cube
        try:
            self.moveit.detach_collision_object(obj_id)
        except:
            pass
        
        self.open()
        
        # 4. Retreat
        self.move_to_pose(make_pose_stamped(tx, ty, CRUISE_Z), True)
        self.retract()
        return True

# ==========================================
#           3. Main task node
# ==========================================
class ReplayNode(Node):
    def __init__(self):
        super().__init__("replay_node")
        self.declare_parameter("task", "replay_two")  # default task

        # Create child nodes for each arm (namespaced)
        self.left_node = rclpy.create_node("driver", namespace="/left_arm")
        self.right_node = rclpy.create_node("driver", namespace="/right_arm")
        
        # Special retract pose for the right arm
        RIGHT_ARM_RETRACT = [0.262, 0.209, 2.094, 1.588, 1.292, 2.042]

        self.left_arm = ArmController(self.left_node, "left_arm", "/left_arm", "left_arm_right_finger_bottom_joint")
        self.right_arm = ArmController(self.right_node, "right_arm", "/right_arm", "right_arm_right_finger_bottom_joint", custom_retract=RIGHT_ARM_RETRACT)

    def task_home(self):
        """Move both arms to home configuration."""
        self.left_arm.move_to_joints(self.left_arm.j_home)
        self.right_arm.move_to_joints(self.right_arm.j_home)

    def task_retract(self):
        """Retract both arms."""
        self.left_arm.retract()
        self.right_arm.retract()

    def task_replay_game(self, limit_steps=None):
        """Replay game script from YAML file."""
        self.get_logger().info(f">>> Start sequence (limit: {limit_steps if limit_steps else 'none'})")
        
        # Search for YAML file in common locations
        filename = 'game_script.yaml'
        possible_paths = [
            os.path.join(os.getcwd(), filename),
            f"/root/roboticsfinalproject/ros2_ws/src/final_moveit/final_moveit/scripts/{filename}"
        ]
        yaml_path = None
        for path in possible_paths:
            if os.path.exists(path):
                yaml_path = path
                break
        
        if not yaml_path:
            self.get_logger().error("game_script.yaml not found! Run generate_game_script.py first.")
            return

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            moves = data.get('moves', [])

        self.get_logger().info(f"Loaded: {len(moves)} moves")
        self.task_retract()

        # Execute move loop
        for i, move in enumerate(moves):
            if limit_steps is not None and i >= limit_steps:
                self.get_logger().info("=== Reached step limit, stopping ===")
                break

            step = move['step']
            arm_role = move['arm']
            grid_idx = move['grid_idx']
            pick_pos = move['pick']
            place_pos = move['place']
            obj_id = move['obj_id']

            self.get_logger().info(f"\n>>> Step {step}: {arm_role} -> Grid {grid_idx}")
            
            if arm_role == 'left':
                self.left_arm.execute_move_sequence(pick_pos, place_pos, obj_id)
            else:
                self.right_arm.execute_move_sequence(pick_pos, place_pos, obj_id)
            
            time.sleep(1.0)

        self.get_logger().info("<<< Sequence complete")

def main(args=None):
    rclpy.init(args=args)
    main_node = ReplayNode()
    
    # Multi-threaded executor for 3 nodes
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(main_node.left_node)
    executor.add_node(main_node.right_node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    # Select task
    task = main_node.get_parameter("task").get_parameter_value().string_value
    
    if task == "home":
        main_node.task_home()
    elif task == "retract":
        main_node.task_retract()
    elif task == "replay_two":
        main_node.task_replay_game(limit_steps=2)
    elif task == "replay_full":
        main_node.task_replay_game(limit_steps=None)
    else:
        main_node.get_logger().warn(f"Unknown task: {task}")

    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
