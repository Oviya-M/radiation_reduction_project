
# 🧠 Radiation Reduction Project – Software Setup & Usage

This document outlines how to replicate the ROS 2-based perception and control pipeline developed for the Elephant Robotics MyCobot 280 M5Stack robotic arm. This setup supports ArUco marker detection, MoveIt2-based motion planning, and real-time servo control.

## 🐘 Cloning & Modifying the Elephant Robotics ROS2 Repository

Our project builds on top of [Elephant Robotics' ROS2 support for the MyCobot 280 M5Stack](https://github.com/elephantrobotics/mycobot_ros2), which includes MoveIt2 integration and launch files for control interfaces.

## 📥 Clone the Repository

Navigate to your workspace’s `src` folder and clone the upstream code:

```
cd ~/ct_bio/src
git clone https://github.com/elephantrobotics/mycobot_ros2.git
```

---

## 🔄 Syncing with GitHub

```
cd ~/ct_bio
git add .
git commit -m "your message"
git pull --rebase origin main
git push origin main
```

---

## ⚙️ Setting Up the Workspace

```
mkdir -p ~/ct_bio/src
cd ~/ct_bio
colcon build --symlink-install
source install/setup.bash
```
---

## 🧵 Speeding Up Builds with Parallel Workers

To accelerate the `colcon build` process, especially on machines with multiple cores, you can use the `--parallel-workers` flag.

## 🔄 Example

```
colcon build --symlink-install --parallel-workers
```

---

## ✅ Prerequisites

Ensure the following are properly installed and configured:

- ROS 2 Humble (`source /opt/ros/humble/setup.bash`)
- Your workspace environment (`source ~/ct_bio/install/setup.bash`)
- Dependencies installed with `colcon build --symlink-install`

---

## 🚀 Launch Sequence

Run each of the following **in separate terminals**, in the listed order:

1. **Start USB Camera Node**

Launches the video stream from a USB webcam.

```
ros2 run usb_cam usb_cam_node_exe
```

2. **Rectify the Camera Image**

Applies image processing and compression for the vision pipeline.

```
ros2 run image_proc rectify_node --ros-args -p image_transport:=compressed -r /image:=/image_raw -r /image/compressed:=/image_raw/compressed
```

3. **Launch MoveIt2 for Motion Planning & Visualization**

This brings up the MoveIt2 environment with RViz.

```
ros2 launch mycobot_280_moveit2 demo.launch.py
```

4. **Start Servo-based Motion Controller**

Enables the robotic arm to respond to pose goals in real-time.

```
ros2 run mycobot_280_moveit2_control sync_plan
```

5. **Run ArUco Marker Detection Subscriber**

This node detects ArUco markers, transforms their poses, and publishes pose goals for the robot.

```
ros2 launch rad_reduction_perception my_subscriber.py
```

---

## 🔧 Workspace & Build Tips

- To build only the perception package:

```
colcon build --packages-select rad_reduction_perception
```

- To rebuild everything:

```
colcon build --symlink-install
```

- If encountering strange behavior:

```bash
rm -rf build/ install/ log/ && colcon build
```

---

## 📐 Robot Constraints

Ensure your robot remains within safe operating limits:

| Joint       | Range           |
|-------------|-----------------|
| Joint 1–5   | -170° to +170°  |
| Joint 6     | -180° to +180°  |

---

## 📸 ArUco Marker Setup

- Use a **10 cm x 10 cm** square marker for consistent detection.
- Make sure the tag is flat and mounted on a rigid surface.
- The `my_subscriber.py` node includes a transform correction for camera orientation and publishes a pose 5 cm above the tag.

---

## 🧪 Troubleshooting Notes

- Reflashing or firmware errors? See [myStudio ROS2 Troubleshooting Guide](https://docs.elephantrobotics.com/docs/mycobot_280_ar_en/3-FunctionsAndApplications/6.developmentGuide/ROS/12.2-ROS2/12.2.5-Moveit2/)
- Common issue: `joint6_flange` transform not broadcasting → check your URDF frames and TF tree.
- Add these `source` lines to your `~/.bashrc` for convenience:

```
source /opt/ros/humble/setup.bash
source ~/ct_bio/install/setup.bash
```

---

## Old Repo:
https://github.com/smw59/ct_bio/tree/main

## Elephant Robotics ROS2 GitHub for MyCobot280_M5
https://github.com/elephantrobotics/mycobot_docs/blob/main/mycobot_280_M5(2023)en/3-FunctionsAndApplications/6.developmentGuide/ROS/12.2-ROS2/12.2.3-ROS2Introduction.md


