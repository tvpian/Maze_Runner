# Maze Runner
## Description
In this ROS2 based implementation, a turtlebot3 performs autonomous waypoint navigation based on the realtime poses of the colored parts present in the gazebo world.

## Dependencies
- ROS2 Galactic
- ArUco marker detection requires the latest version of OpenCV
```bash
  - pip3 uninstall opencv-python
  - pip3 uninstall opencv-contrib-python
  - pip3 install opencv-contrib-python
  - pip3 install opencv-python
```
- To install ROS packages for this demo, it is recommended to create a new
workspace.
```bash
  - mkdir -p ~/maze_ws/src
  - cd ~/maze_ws/src
  - git clone https://github.com/tvpian/Maze_Runner.git
  - rosdep install --from-paths src -y --ignore-src
  - colcon build
```

## Usage
- [Link to usage](./group1_final/readme.md)
- Note: Considering all the packages including the group1_final is cloned in the same workspace, Please follow only the instructions under the "Running the Project
" section in the link above.

[demo.webm](https://github.com/tvpian/Maze_Runner/assets/41953267/9c2e4b79-4b36-48e2-995b-bce9027824f6)
