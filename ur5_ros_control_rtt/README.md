

launch control nodes

```
roslaunch ur5_ros_control ur5_ros_control.launch
```
```
deployer-gnulinux -s ur5_ros_control_rtt.ops
```

launch moveit
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```
```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
