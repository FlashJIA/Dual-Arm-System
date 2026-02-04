#!/usr/bin/env python3
"""
Lab 6 — Pick & Place (Kinova Gen3 Lite) - Student Version

This script provides a basic "pick and place" task where the robot
picks up a block, lifts it, and places it back in the same spot.

You can extend this to stack blocks in different locations.

Quick start:
  # First, in a new terminal, add the scene
  ros2 run lab06_moveit pick_and_place --ros-args -p task:=add_scene

  # Then, run the main task
  ros2 run lab06_moveit pick_and_place --ros-args -p task:=pick_place_one
"""
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface

# Reusable function to create a Pose message
def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Declare a ROS parameter for the task to be performed
        self.declare_parameter("task", "home")

        # --- Robot Configuration ---
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.01],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=True,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        # --- Scene and Pose Definitions ---
        table_z = -0.0001
        block_size = 0.04
        self.table_center_z = table_z - 0.05/2.0
        self.block_center_z = table_z + block_size/2.0

        block_x = 0.45
        self.blocks_xyz = [
            (block_x, -0.08, self.block_center_z),
            (block_x,  0.00, self.block_center_z),
            (block_x,  0.08, self.block_center_z),
        ]

        self.approach_height = 0.32
        self.grasp_height = 0.19
        self.top_down_orientation = (-0.7071, 0.7071, 0.0, 0.0)

        # --- Define Poses for the Pick and Place Task ---
        # 1. Poses for picking up the first block
        pick_xyz = self.blocks_xyz[0]
        self.pre_grasp_pose = make_pose(pick_xyz[0], pick_xyz[1], self.approach_height, *self.top_down_orientation)
        self.grasp_pose     = make_pose(pick_xyz[0], pick_xyz[1], self.grasp_height,    *self.top_down_orientation)

        # 2. Poses for placing the block back down
        # For this basic lab, the place location is the same as the pick location.
        # Students will modify these poses to stack the block elsewhere.
        self.pre_place_pose = self.pre_grasp_pose
        self.place_pose     = self.grasp_pose

        self.j_home    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]
        self.touch_links = ["end_effector_link",
                            "right_finger_bottom_link", "left_finger_bottom_link"]

    # ---- Wrapper Functions for Robot Actions ----
    def move_to_joints(self, joint_positions):
        self.moveit2.move_to_configuration(joint_positions=joint_positions)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        traj = self.moveit2.plan(
            pose=pose,
            cartesian=cartesian,
            max_step=0.005,
            cartesian_fraction_threshold=0.90 if cartesian else None,
        )
        if traj is None:
            self.get_logger().error("Planning failed.")
            return False
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        return True

    def open_gripper(self):
        if not self.gripper: return
        self.gripper.open()

    def close_gripper(self):
        if not self.gripper: return
        self.gripper.close()

    # ---- Scene Management ----
    def add_scene(self):
        self.get_logger().info("Adding objects to the planning scene.")
        self.moveit2.add_collision_box(
            id="table_top",
            size=(1.0, 1.0, 0.05),
            position=(0.0, 0.0, self.table_center_z),
            quat_xyzw=(0,0,0,1),
            frame_id="base_link",
        )
        time.sleep(0.1)
        for i, (x,y,z) in enumerate(self.blocks_xyz, start=1):
            self.moveit2.add_collision_box(
                id=f"block_{i}",
                size=(0.04, 0.04, 0.04),
                position=(x, y, z),
                quat_xyzw=(0,0,0,1),
                frame_id="base_link",
            )
            time.sleep(0.05)
        self.get_logger().info("Planning scene has been set up.")

    # ---- Main Task ----
    def task_pick_place_one(self):
        """Execute a simple sequence: pick up a block, lift it, and place it back down."""
        self.get_logger().info("Starting pick and place task for 'block_1'.")

        # 1. Initial Setup
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(1.0) # Give the gripper time to open

        # 2. Pick Sequence
        self.get_logger().info("--- PICK SEQUENCE ---")
        # 2a. Move to a "pre-grasp" pose above the block
        if not self.move_to_pose(self.pre_grasp_pose): return
        # 2b. Move straight down to the "grasp" pose
        if not self.move_to_pose(self.grasp_pose, cartesian=True): return

        # 2c. Close the gripper and attach the object in the planning scene
        self.get_logger().info("Closing gripper to grasp block...")
        self.close_gripper()
        time.sleep(1.0) # Wait for contacts to settle
        self.moveit2.attach_collision_object("block_1", "end_effector_link", self.touch_links)
        time.sleep(0.2)

        # 2d. Move straight up to the "pre-grasp" pose to lift the block
        if not self.move_to_pose(self.pre_grasp_pose, cartesian=True): return

        # 3. Place Sequence
        self.get_logger().info("--- PLACE SEQUENCE ---")
        # 3a. Move straight down to the "place" pose (which is the same as the grasp pose)
        if not self.move_to_pose(self.place_pose, cartesian=True): return

        # 3b. Open the gripper and detach the object from the planning scene
        self.open_gripper()
        time.sleep(1.0) # Wait for the object to release
        self.moveit2.detach_collision_object("block_1")
        time.sleep(0.2)

        # 4. Completion
        # 4a. Move up away from the placed block
        _ = self.move_to_pose(self.pre_place_pose, cartesian=True)
        # 4b. Return to a final retracted pose
        self.move_to_joints(self.j_retract)
        self.get_logger().info("Pick and place task completed successfully.")

    # ---- Simple Task Callbacks ----
    def task_home(self): self.move_to_joints(self.j_home)
    def task_retract(self): self.move_to_joints(self.j_retract)
    def task_add_scene(self): self.add_scene()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if task == "home":             node.task_home()
        elif task == "retract":        node.task_retract()
        elif task == "add_scene":      node.task_add_scene()
        elif task == "pick_place_one": node.task_pick_place_one()
        else:
            node.get_logger().warn(f"Unknown task '{task}'. Valid tasks: home, retract, add_scene, pick_place_one")
    finally:
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()