# Maze Runner
## Description
In this ROS2 based implementation, a turtlebot3 performs autonomous waypoint navigation based on the realtime poses of the colored parts present in the gazebo world.

## Dependencies
- ROS2 Galactic
- ArUco marker detection requires the latest version of OpenCV
  - pip3 uninstall opencv-python
  - pip3 uninstall opencv-contrib-python
  - pip3 install opencv-contrib-python
  - pip3 install opencv-python
- To install ROS packages for this demo, it is recommended to create a new
workspace.
    - mkdir -p ~/maze_ws/src
    - cd ~/maze_ws/src
    - git clone https://github.com/tvpian/Maze_Runner.git
    - rosdep install --from-paths src -y --ignore-src
    - colcon build
  