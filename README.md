# udacity_robotic_software_engineer

This repo contains all the projects done for the [Udacity Robotic Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209) course.


## Getting Started

### Dependencies

Here is the list of software should be already installed:

- gazebo
- ros-kinetic
- gcc
- catkin


### Setup Catkin

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```

**Troubleshoot**
If any errors were encounted refer to the [official docs](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Launch Projects

### Gazebo
In one terminal run:
```bash
cd project1_gazebo
mkdir build
cd build
cmake 
gazebo world/myworld.world
```


### GoChaseIt

Build the project:
```bash
cd project2_gochaseit
ln -s -t ~/catkin_ws/src/ ball_chaser
ln -s -t ~/catkin_ws/src/ my_robot
cd ~/catkin_ws
catkin_make
```
Now run it:

1. In one terminal run:
```bash
cd ~/catkin_ws
source source devel/setup.zsh
roslaunch my_robot world.launch
```

2. On another terminal run:
```bash
cd ~/catkin_ws
source source devel/setup.zsh
roslaunch ball_chaser ball_chaser.launch
```

## Licence

GNU GPLv3 