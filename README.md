# kinect-process-scene
TODO: Clean this shit up, add module for segmentation/clustering/object pose estimation

Collection of scripts and methods used to process table top scenes for object recognition/detection/segmentation using the Kinect sensors (Xbox 360, One) in ROS.

######Dependencies:
| Dependencies  |
| ------------- |
| [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation)           |
| [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages)    |
| [freenect](http://wiki.ros.org/freenect_launch) |
| [openni_launch](https://github.com/ros-drivers/openni_launch) |
| [record-ros](https://github.com/epfl-lasa/record_ros)|
|[PCL library](http://www.pointclouds.org) |
|[OpenCV] (http://www.opencv.org)|
---
####Capture real kinect images in realtime:

#####Launch Freenect Drivers and Registration Nodes:
```
$ roslaunch freenect_launch freenect-registered-xyzrgb.launch
```

#####Run KUKA Bridge to stream joint data:
```
$ rosrun kuka_fri_bridge run_lwr.sh
```

#####To visualize real kinect images + joint data:
######Launch rviz
```
$ roslaunch kuka_lwr_bringup lwr2_alone_realtime.launch robot_urdf_name:=kuka_grav_comp.xacro
