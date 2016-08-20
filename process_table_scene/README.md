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
$ roslaunch kuka_lwr_bringup lwr2_tabletop.launch robot_urdf_name:=kuka_grav_comp.xacro
