#!/usr/bin/env python3
import time
import threading
import random
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from control_msgs.action import GripperCommand

# --- Configuration parameters ---
TOP_DOWN_QUAT = (-0.7071, 0.7071, 0.0, 0.0)
# BOARD_CENTER_X = 0.46 0.41
BOARD_CENTER_X = 0.42

SPACING = 0.08

# 🖐️ Manually tuned wrist height for picking
TEST_WRIST_Z = 0.485
# Cruise height (safe height above table)
CRUISE_Z = TEST_WRIST_Z + 0.10
# Placement height offset for tile thickness
TILE_OFFSET = 0.002
PLACE_WRIST_Z = TEST_WRIST_Z + TILE_OFFSET

def make_pose_stamped(x, y, z, frame_id="world") -> PoseStamped:
    """Helper to create a PoseStamped in the world frame with a fixed top-down orientation."""
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

class ArmController:
    # ▼▼▼ Key feature: support custom_retract so different arms can have different retract poses ▼▼▼
    def __init__(self, ros_node, arm_name, namespace, gripper_joint_name, custom_retract=None):
        self.node = ros_node
        self.arm_name = arm_name
        self.gripper_joint_name = gripper_joint_name
        
        # Basic MoveIt2 setup for a 6-DoF arm
        self.moveit = MoveIt2(
            node=self.node,
            joint_names=[f"{arm_name}_joint_{i}" for i in range(1, 7)],
            base_link_name=f"{arm_name}_base_link",
            end_effector_name=f"{arm_name}_end_effector_link",
            group_name="arm",
        )

        # Gripper action client (same namespace as arm)
        self.gripper_client = ActionClient(
            self.node,
            GripperCommand,
            f"{namespace}/gen3_lite_2f_gripper_controller/gripper_cmd"
        )

        # Home joint configuration (all zeros)
        self.j_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # ▼▼▼ Set retract pose ▼▼▼
        if custom_retract:
            # Use a custom retract pose if provided
            self.j_retract = custom_retract
        else:
            # Default retract pose (tuned for the left arm)
            # self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]
            self.j_retract = [0.471, 0.332, 2.112, -1.571, -1.292, 2.042]

        # Approximate block size for collision box
        self.block_size = 0.04
        self.add_table_collision()

    def add_table_collision(self):
        """Add a collision box representing the table surface."""
        try:
            self.moveit.add_collision_box(
                "table_ground",
                (2.0, 1.0, 0.29),
                (0.5, 0.0, 0.145),
                (0, 0, 0, 1),
                "world"
            )
        except:
            pass

    def move_to_joints(self, joint_positions):
        """Move arm to a given joint configuration and wait for completion."""
        self.moveit.move_to_configuration(joint_positions=joint_positions)
        self.moveit.wait_until_executed()

    def move_to_pose(self, pose, cartesian=False):
        """
        Plan and execute a motion to a target pose.

        cartesian=False: use standard MoveIt planning.
        cartesian=True : use Cartesian path planning.
        """
        traj = self.moveit.plan(pose=pose, cartesian=cartesian, max_step=0.005)
        if traj:
            self.moveit.execute(traj)
            self.moveit.wait_until_executed()
            return True
        else:
            self.node.get_logger().error(f"[{self.arm_name}] Motion planning failed!")
            return False

    def _send_gripper_command(self, position, wait_time=0.0):
        """Send a gripper action command and optionally block for a given wait time."""
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.arm_name}] Gripper controller not available!")
            return
            
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 100.0
        self.gripper_client.send_goal_async(goal)
        
        if wait_time > 0:
            time.sleep(wait_time)

    # --- Your tuned gripper parameters ---
    def open_gripper(self):
        """Open the gripper to a wide angle (blocking wait for motion)."""
        self._send_gripper_command(0.45, wait_time=5.5)

    def close_gripper_blocking(self):
        """Close the gripper to grasp an object (blocking wait for motion)."""
        self._send_gripper_command(0.70, wait_time=5.5)

    def retract(self):
        """
        Retract routine:
        1) Set gripper to half-closed.
        2) Move arm to its retract joint configuration.
        """
        self._send_gripper_command(0.40, wait_time=0.2)
        self.move_to_joints(self.j_retract)

    def test_gripper(self, target_value):
        """Move the gripper to an arbitrary target value (no blocking wait)."""
        self._send_gripper_command(float(target_value))

    def pick(self, x, y, z_wrist_height, object_id):
        """Full pick sequence: approach, grasp, attach collision, and lift."""
        self.node.get_logger().info(f"[{self.arm_name}] PICK -> {object_id}")
        
        # Add a collision box for the object at a fixed table height
        try:
            self.moveit.add_collision_box(
                object_id,
                (self.block_size,) * 3,
                (x, y, 0.32),
                (0, 0, 0, 1),
                "world"
            )
        except:
            pass

        # Ensure gripper is open before approaching the object
        self.open_gripper()
        
        # Pre-grasp pose (above the object)
        if not self.move_to_pose(make_pose_stamped(x, y, z_wrist_height + 0.10), False):
            return False
        # Grasp pose (at wrist height)
        if not self.move_to_pose(make_pose_stamped(x, y, z_wrist_height), True):
            return False 
        
        # Close gripper to grasp the object
        self.close_gripper_blocking()
        
        # Attach the collision object to the gripper links so it moves with the arm
        try:
            self.moveit.attach_collision_object(
                object_id,
                f"{self.arm_name}_end_effector_link",
                [
                    f"{self.arm_name}_right_finger_bottom_link",
                    f"{self.arm_name}_left_finger_bottom_link"
                ]
            )
        except:
            pass

        # Lift the object up after grasping
        if not self.move_to_pose(make_pose_stamped(x, y, z_wrist_height + 0.15), True):
            return False
        return True

    def place(self, x, y, z_wrist_height, object_id):
        """Full place sequence: approach, lower, release, detach object, and retreat."""
        self.node.get_logger().info(f"[{self.arm_name}] PLACE -> {object_id}")
        
        # Pre-place pose (above target location)
        if not self.move_to_pose(make_pose_stamped(x, y, z_wrist_height + 0.10), False):
            return False
        # Place pose (lower to the placement height)
        if not self.move_to_pose(make_pose_stamped(x, y, z_wrist_height), True):
            return False
        
        # Detach object from the gripper in the planning scene
        try:
            self.moveit.detach_collision_object(object_id)
        except:
            pass
        
        # Open gripper to release the object
        self.open_gripper()
        
        # Retreat to a safe height after placing
        self.move_to_pose(make_pose_stamped(x, y, z_wrist_height + 0.10), True)
        return True


class DualArmTasks(Node):
    """High-level task node coordinating both left and right arms."""

    def __init__(self):
        super().__init__("dual_arm_task_node")
        # Parameter to choose which task to run (home, retract, pick, etc.)
        self.declare_parameter("task", "home")

        # Create separate ROS nodes for each arm under different namespaces
        self.left_node = rclpy.create_node("driver", namespace="/left_arm")
        self.right_node = rclpy.create_node("driver", namespace="/right_arm")

        # ▼▼▼ Right arm specific retract pose (from your tuned data) ▼▼▼
        # RIGHT_ARM_RETRACT = [0.175, 0.052, 2.094, 1.588, 1.134, 1.955]
        # Replace RIGHT_ARM_RETRACT = [...] line with:
        RIGHT_ARM_RETRACT = [0.262, 0.209, 2.094, 1.588, 1.292, 2.042]

        # Left arm controller (uses default retract pose)
        self.left_arm = ArmController(
            self.left_node, "left_arm", "/left_arm", "left_arm_right_finger_bottom_joint"
        )
        
        # Right arm controller (uses custom retract pose)
        self.right_arm = ArmController(
            self.right_node, "right_arm", "/right_arm", "right_arm_right_finger_bottom_joint",
            custom_retract=RIGHT_ARM_RETRACT
        )

        # Containers for detected object positions (from YAML)
        self.red_cubes = []
        self.blue_cylinders = []
        self.load_objects_from_yaml()
        self.red_idx = 0
        self.blue_idx = 0

    def load_objects_from_yaml(self):
        """Load detected object positions from a YAML file."""
        filename = 'detected_objects.yaml'
        possible_paths = [
            os.path.join(os.getcwd(), filename),
            f"/root/workspace/sj473_robotics_fall2025/final/ros2_ws/src/final_moveit/final_moveit/scripts/{filename}"
        ]
        yaml_path = None
        for path in possible_paths:
            if os.path.exists(path):
                yaml_path = path
                break
        if yaml_path:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.red_cubes = data.get('red_cubes', [])
                self.blue_cylinders = data.get('blue_cylinders', [])
            self.get_logger().info(
                f"Loaded detection data: red={len(self.red_cubes)}, blue={len(self.blue_cylinders)}"
            )

    def task_home(self):
        """Move both arms to their home configurations."""
        self.left_arm.move_to_joints(self.left_arm.j_home)
        self.right_arm.move_to_joints(self.right_arm.j_home)

    def task_retract(self):
        """Retract both arms to their retract poses."""
        self.left_arm.retract()
        self.right_arm.retract()

    # ▼▼▼ Task: left arm places a red cube at the top-right cell (Row 0, Col 2) ▼▼▼
    def task_pick_cruise_place(self):
        """Left arm: pick one red cube and place it at the top-right grid cell."""
        if not self.red_cubes:
            return
        # Start from retract pose for both arms
        self.task_retract()
        target = self.red_cubes.pop(0)
        self.red_idx += 1
        obj_id = f"red_block_{self.red_idx}"
        pick_x, pick_y, _ = target 
        
        # Target grid index: row=0, col=2 (top-right)
        row = 0
        col = 2
        place_y = (1 - row) * SPACING
        place_x = BOARD_CENTER_X + (col - 1) * SPACING

        if self.left_arm.pick(pick_x, pick_y, TEST_WRIST_Z, obj_id):
            # Move up to cruise height at pick location
            self.left_arm.move_to_pose(make_pose_stamped(pick_x, pick_y, CRUISE_Z), True)
            # Move in free-space to cruise height above place location
            self.left_arm.move_to_pose(make_pose_stamped(place_x, place_y, CRUISE_Z), False)
            # Use PLACE_WRIST_Z for placement height
            self.left_arm.place(place_x, place_y, PLACE_WRIST_Z, obj_id)
            # Retract after placing
            self.left_arm.retract()

    # ▼▼▼ Task: right arm places a blue cylinder at the top-left cell (Row 0, Col 0) ▼▼▼
    def task_pick_cruise_place_blue(self):
        """Right arm: pick one blue cylinder and place it at the top-left grid cell."""
        if not self.blue_cylinders:
            return
        # Start from retract pose for both arms
        self.task_retract()
        target = self.blue_cylinders.pop(0)
        self.blue_idx += 1
        obj_id = f"blue_cyl_{self.blue_idx}"
        pick_x, pick_y, _ = target 
        
        # Target grid index: row=0, col=0 (top-left, farthest corner)
        row = 0
        col = 0
        place_y = (1 - row) * SPACING
        place_x = BOARD_CENTER_X + (col - 1) * SPACING

        if self.right_arm.pick(pick_x, pick_y, TEST_WRIST_Z, obj_id):
            # Move up to cruise height at pick location
            self.right_arm.move_to_pose(make_pose_stamped(pick_x, pick_y, CRUISE_Z), True)
            # Move in free-space to cruise height above place location
            self.right_arm.move_to_pose(make_pose_stamped(place_x, place_y, CRUISE_Z), False)
            # Use PLACE_WRIST_Z for placement height
            self.right_arm.place(place_x, place_y, PLACE_WRIST_Z, obj_id)
            # Retract after placing
            self.right_arm.retract()

    def task_pick_red_return(self):
        """Left arm: pick a red cube and return it to (approximately) the same location."""
        if not self.red_cubes:
            return
        self.task_retract()
        target = self.red_cubes.pop(0)
        self.red_idx += 1
        obj_id = f"red_block_{self.red_idx}"
        x, y, _ = target 
        if self.left_arm.pick(x, y, TEST_WRIST_Z, obj_id):
            self.left_arm.place(x, y, TEST_WRIST_Z, obj_id)
            self.left_arm.retract()

    def task_pick_blue_return(self):
        """Right arm: pick a blue cylinder and return it to (approximately) the same location."""
        if not self.blue_cylinders:
            return
        self.task_retract()
        target = self.blue_cylinders.pop(0)
        self.blue_idx += 1
        obj_id = f"blue_cyl_{self.blue_idx}"
        x, y, _ = target 
        if self.right_arm.pick(x, y, TEST_WRIST_Z, obj_id):
            self.right_arm.place(x, y, TEST_WRIST_Z, obj_id)
            self.right_arm.retract()

    def task_test_gripper_value(self):
        """Send a test command to both grippers (for calibration/inspection)."""
        self.left_arm.test_gripper(0.75)
        self.right_arm.test_gripper(0.75)

def main(args=None):
    """Entry point: create node, spin executor in a thread, dispatch task by parameter."""
    rclpy.init(args=args)
    main_node = DualArmTasks()
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(main_node.left_node)
    executor.add_node(main_node.right_node)
    
    # Spin in a background thread so we can still run blocking tasks in main thread
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    # Read which task to execute from ROS parameter
    task = main_node.get_parameter("task").get_parameter_value().string_value
    
    if task == "home":
        main_node.task_home()
    elif task == "retract":
        main_node.task_retract()
    elif task == "pick_red_return":
        main_node.task_pick_red_return()
    elif task == "pick_blue_return":
        main_node.task_pick_blue_return()
    elif task == "pick_cruise_place":
        main_node.task_pick_cruise_place()
    elif task == "pick_cruise_place_blue":
        main_node.task_pick_cruise_place_blue()
    elif task == "test_gripper":
        main_node.task_test_gripper_value()
    else:
        main_node.get_logger().warn(f"Unknown task: {task}")

    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
