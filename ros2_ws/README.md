1.Pereqs
sudo apt-get update
sudo apt-get install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
                     ros-jazzy-depth-image-proc ros-jazzy-image-pipeline

2.Folder layout
~/workspace/[]_robotics_fall2025/lab06/ros2_ws/src/sim_cam/sdf/depth_cam.sdf
mkdir -p ~/workspace/[]_robotics_fall2025/lab06/ros2_ws/src/sim_cam/sdf
inside the new folder sdf create new file
depth_cam.sdf

3) Start your world

4) Spawn the camera (top-down)
ros2 run ros_gz_sim create \
  -file ~/workspace/sj473_robotics_fall2025/lab06/ros2_ws/src/sim_cam/sdf/depth_cam.sdf \
  -name sim_depth_camera \
  -x 0.3 -y 0 -z 2 \
  -R 0 -P 1.5708 -Y 0

xyz and orientation

5) Bridge Gazebo topics to ROS
In a new terminal:
ros2 run ros_gz_bridge parameter_bridge \
  /sim/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /sim/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image \
  /sim/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

6) Visualize in RViz2

Add Image:

Topic: /sim/camera/image (RGB)

Add Image:

Topic: /sim/camera/depth_image (Depth)

Turn on Normalize Range (otherwise depth can look black/white)

(Optional, recommended) Convert depth to a point cloud:

ros2 run depth_image_proc point_cloud_xyz \
  --ros-args \
  -r image:=/sim/camera/depth_image \
  -r camera_info:=/sim/camera/camera_info \
  -p range_max:=5.0