### Home Service Project

This project covered a full robot SLAM project.

1- First I created a map launched with `test_slam.sh`. It utilizes the `gmapping` package for reading the sensor and creating the map.
```bash
./test_slam.sh
```
2- I saved this map under the `map` folder with the `map_server` tool
```bash
rosrun map_server map_server ../map/mapped_myworld.yaml
```
3- I used the generated `map` in combination with the `amcl` package for localizing and navigating to a set goal with Rviz.
```bash
./test_navigation.sh
```
4- I created a virtual object/marker following the tutorial. This marker is visualized in Rviz
```bash
./add_marker.sh
```
5- The last test was set up a goal with the `SimpleActionClient` that belongs to the `actionlib`. The navigation package handles the server-side, for executing the path planning for reaching the goal.
```bash
./pick_objects.sh
```
6- Finally the `home_service.sh` is executed, where the marker and the picker node should be connected via the specific messages and fulfill the task> 
```bash
./home_service.sh
```