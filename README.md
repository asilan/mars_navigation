# Mars Navigation Package

This ROS package provides navigation capabilities for Mars rover simulation.

## Dependencies

- ROS (tested on ROS Noetic)
- roscpp
- std_msgs
- nav_msgs
- geometry_msgs

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

To launch the navigation node:

```bash
roslaunch mars_navigation mars_navigation.launch
```

## Package Structure

- `src/`: Source files
- `include/`: Header files
- `launch/`: Launch files
- `config/`: Configuration files
