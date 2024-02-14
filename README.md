# **PTP Trajectory package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Plan fixed point-to-point trajectory for path tracking and localization testing. 
___

`ptp_trajectory` node computes fixed pre-defined trajectory based on parameters given in the service message. Trajectory points are computed with fixed step. Allows for path_tracking and localization (either pure visual odometry or odometry fusion) testing without dependency on the cone detection, SLAM or path planning.

### Services
* `SetTarget.srv` : define 2D coordinates for the target to be reached by straight line trajectory
* `GoRectangle.srv` : define dimensions of a rectangle to be passed (starting in the middle of the side "a") and direction of the turn.

## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build ptp_trajectory
```

### Compilation configuration
* [`ptp_trajectory.h`](./include/ptp_trajectory.h)
	* `WAYPOINT_DISTANCE` : [m]; trajectory points step
	* `MIN_DISTANCE` : [m]; radius for reaching the target

## Launch
### Source the environment
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
```
### Launch the node
* standalone
```sh
$ rosrun ptp_trajectory ptp_trajectory
```
* along with `path_tracking` and `camera_driver` (for visual odometry) on RC car
```sh
$ roslaunch master rc_test.launch
```

### Send a command
Edit the data fields according to your requirement. If the `path_tracking` node and the `odometry_interface` topic are active, the vehicle will start to move immediately after sending the command.

* plan straight trajectory
```sh
$ rosservice call /ptp_trajectory/set_target "coords:
  x: 0.0
  y: 0.0 "
```
* plan rectangle trajectory
```sh
$ rosservice call /ptp_trajectory/go_rectangle "a: 0.0
  b: 0.0
  right: 0" 
```
