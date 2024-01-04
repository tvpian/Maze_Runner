# ENPM809Y Final Project - Group 1
final project for ENPM809Y - Fall 2023

# Authors
- Obaid Ur Rehman - Developer
- Rachit Thakur - Developer
- Tharun V. Puthanveettil - Maintainer

# Dependencies
ROS2 Galactic is used for this project. The installation instructions for ROS2 Galactic can be found [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). The following commands can be used to install ROS2 Galactic on Ubuntu 20.04:
```bash
 sudo apt update
 sudo apt install curl gnupg2 lsb-release
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
 sudo apt update
 sudo apt install ros-galactic-desktop
```


# Build Instructions
- Unzip the [group1_final.zip](./group1_final.zip) file into the src folder of your ROS2 workspace.
- Build the project using the following commands:
    ```bash
    cd <path_to_your_ros2_workspace>
    colcon build --packages-select group1_final
    ```

# Running the Project
- The project can be run using the following command:
- Run the launch file to start the simulation provided:
    ```bash
    ros2 launch final_project final_project.launch.py
    ```
- Run the node to start the broadcasting of static frames node:
    ```bash
    ros2 run group1_final static_broadcaster_node --ros-args -p use_sim_time:=True
    ```
- Run the node to perform part detection and waypoint navigation:
    ```bash
    ros2 run group1_final aruco_detector_node --ros-args -p use_sim_time:=True --params-file <path-to-config>.yaml
    ```   
    eg:
    ```bash
    ros2 run group1_final aruco_detector_node --ros-args -p use_sim_time:=True --params-file /home/tvp/TVP/Coursework/Fall2023/ENPM809Y/Project/final_ws/src/group1_final/config/waypoint_params.yaml
    ```
Note: There will be a wait time of 20 secs before the robot starts moving. This is to ensure that the initial pose of the robot is correctly set in the map frame.


# References
- [ROS2 Documentation](https://docs.ros.org/en/galactic/index.html)
- [ROS2 Navigation2](https://navigation.ros.org/index.html)
- [ROS2 Navigation2 Tutorials](https://navigation.ros.org/getting_started/index.html)
- ENPM 809Y Course Material
