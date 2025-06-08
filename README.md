# Waypoint Navigation Manager for Mobile Robot (ROS 2)

## Overview

This ROS 2 C++ project enables a mobile robot to record and manage waypoints during manual navigation. Waypoints can be created both manually by the user and automatically by the robot based on motion behavior such as distance traveled or angle turned. These waypoints are stored with unique identifiers and can be persisted to disk for later use in autonomous navigation tasks.

### Key Features
📍 Manual waypoint creation with unique IDs

🤖 Automatic waypoint creation based on distance/angle thresholds

🔁 Deduplication based on proximity to existing waypoints

💾 YAML-based waypoint storage (position, orientation, ID, type)

🧭 Custom utility to extract yaw from robot orientation

📊 RViz visualization using MarkerArray

## Dependencies

- ROS 2 Humble (or later)
- `nav2_msgs`
- `geometry_msgs`
- `custom_msgs` (custom message package containing `MapNavigation.action`)

## [Demonstration] : (https://drive.google.com/file/d/1ZQMmcTyLOfUn8IGfA0jcelHAxV9yFzJX/view?usp=sharing)



## Steps to run



1. Clone the repo 
```
git clone https://github.com/nivednivu1997/Waypoint_Manager.git
```
2. Install Dependencies
```
rosdep install --from-path src --ignore-src -y
```
3. Building Package
```
 cd Waypoint_Manager && colon build --symlink-install
```


## Steps to run 

1. Launch the gazebo simulation
```
cd Waypoint_Manager && source install/setup.bash && ros2 launch butlerbot_rmf_gazebo simulation.launch.py
```
2. Launch the navigation2 stack 
```
cd Waypoint_Manager && source install/setup.bash && ros2 launch butlerbot_navigation navigation.launch.py
```
3. Run teleop keyboard
```
 ros2 run teleop_twist_keyboard teleop_twist_keyboard 

```
4. Run the waypoint node
```
cd Waypoint_Manager && source install/setup.bash && ros2 run cpp_node waypoint_node

```
Manual Wypoints can be created by
```
ros2 service call /create_manual_waypoint std_srvs/srv/Trigger

```
## Acknowledgments  

This project utilizes the Gazebo simulation setup from [ROS-Assignment](https://github.com/manojm-dev/ROS-Assignment).  




