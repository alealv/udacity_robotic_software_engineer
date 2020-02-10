# udacity_robotic_software_engineer


## Setup Catkin

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Link project files

```bash
ln -s `pwd`/project2_gochaseit
cd ~/catkin_ws
catkin_make
```

## Launch GO_CHASE_IT

In one terminal run:
```bash
cd ~/catkin_ws
source source devel/setup.zsh
roslaunch my_robot world.launch
```

On another terminal run:
```bash
cd ~/catkin_ws
source source devel/setup.zsh
roslaunch ball_chaser ball_chaser.launch
```