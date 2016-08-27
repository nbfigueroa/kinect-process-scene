# kinect-process-scene
Collection of scripts and methods used to process table top scenes for object recognition/detection/segmentation using the Kinect sensors v1 (Xbox 360) and v2 (One) in ROS.

| Main Dependencies  |
| ------------- |
| [PCL library](http://www.pointclouds.org) |
| [OpenCV] (http://www.opencv.org)|

When using Kinect v1 need to install: [openni_launch](https://github.com/ros-drivers/openni_launch) and/or [freenect](http://wiki.ros.org/freenect_launch).

When using  Kinect v2 need to install: [iai_kinect2](https://github.com/code-iai/iai_kinect2)

---

####kinect2_receiver
My own Kinect2_receiver class which processes pre-registered depth and color images from kinect2-bridge

####process_table_scene
"Robot-aware" table-top scene processing. 
- Filters points generated by robots/tools. 
- Segments table and desired object.

####object_feature_generator
Extracts and streams default object features (from segmented point cloud)
- mean & std of r,g,b values (as a wrench message)
---
###Bimanual Peeling Zucchini Segmentor Instructios

####On lasapc18
Start-up Kinect2 driver :
```
$ roslaunch kinect2_bridge kinect2_bridge.launch
```

####On lasa-beast
Generate PointCloud on local machine from depth/color images sent through Network:
```
$ rosrun kinect2_receiver publisher_node
```
Run Zucchini segmentation node (publishes zucchini and cutting board point clouds):
```
$ roslaunch process_table_scene bimanual_scene.launch
```
Generate Object Features Online (Publishes Zucchini Color Features):
```
$ rosrun object_feature_generator feature_generator_node
```
