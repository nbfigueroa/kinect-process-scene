# Kinect2 Receiver

My own `Kinect2_receiver` class which processes pre-registered depth and color images from [kinect2-bridge](https://github.com/code-iai/iai_kinect2)

## Description

This class listens to two ROS topics (depth and color + camera info) and has the functionalites of displaying the registered point cloud and/or publishing the point cloud.

## Dependencies

- ROS Hydro/Indigo
- OpenCV (2.8-ish, fucks up with 3)
- PCL (1.7-ish)

# Usage
After launching the kinect2_bridge on any machine in the network as:
```
roslaunch kinect2_bridge kinect2_bridge.launch
```

You can subscribe to the low-resolution depth and color images and view/publish the registered point cloud with:

Example: `rosrun kinect2_viewer viewer_node`
Example: `rosrun kinect2_viewer publisher_node`

## Key bindings

Window Viewer:
- `ESC`, `q`: Quit

Terminal:
- `CRTL`+`c`: Quit
