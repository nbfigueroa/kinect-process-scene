# Kinect2 Receiver

My own Kinect2_receiver class following iai_kinect2.

## Description

This class listens to two ROS topics (depth and image + camera info) and has the functionalites of displaying the registered point cloud and/or publishing the point cloud.

## Dependencies

- ROS Hydro/Indigo
- OpenCV (2.8-ish, fucks up with 3)
- PCL (1.7-ish)


Example: `rosrun kinect2_viewer viewer_node`
Example: `rosrun kinect2_viewer publisher_node`

## Key bindings

Window Viewer:
- `ESC`, `q`: Quit

Terminal:
- `CRTL`+`c`: Quit
