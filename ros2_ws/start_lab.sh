#!/bin/bash

# Automatic environment setup
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "=========================================="
echo "    Waiting for Gazebo and Controller Manager..."
echo "    (Please run 'ros2 launch' in another terminal)"
echo "=========================================="

# 1. Loop wait for Left Arm to come online
until ros2 service list | grep -q "/left_arm/controller_manager/load_controller"; do
  printf "."
  sleep 2
done
echo -e "\n✅ Left Arm is ready! Loading controllers..."

# 2. Activate Left Arm controllers (Parallel execution)
ros2 run controller_manager spawner joint_state_broadcaster -c /left_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
ros2 run controller_manager spawner joint_trajectory_controller -c /left_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
ros2 run controller_manager spawner gen3_lite_2f_gripper_controller -c /left_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

# 3. Loop wait for Right Arm to come online
until ros2 service list | grep -q "/right_arm/controller_manager/load_controller"; do
  printf "."
  sleep 2
done
echo -e "\n✅ Right Arm is ready! Loading controllers..."

# 4. Activate Right Arm controllers
ros2 run controller_manager spawner joint_state_broadcaster -c /right_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
ros2 run controller_manager spawner joint_trajectory_controller -c /right_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
ros2 run controller_manager spawner gen3_lite_2f_gripper_controller -c /right_arm/controller_manager --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

echo "=========================================="
echo "    🎉 All controllers have been sent start commands!"
echo "=========================================="