#ifndef KINECT2_RECEIVER_H
#define KINECT2_RECEIVER_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


class Kinect2_Receiver
{
public:

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  std::string topicCloud, cloudFrame;
  const bool useCompressed;

  bool updateCloud, publishCloud, withViewer;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  std::thread imageViewerThread;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  std::ostringstream oss;
  std::vector<int> params;
  ros::Publisher pubCloud;

public:


  Kinect2_Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useCompressed);

  ~Kinect2_Receiver();

  void run_viewer();

  void run_publisher(const std::string &topicCloud, const bool withViewer);


private:
  void start();

  void stop();

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

  void generateCloud();

  void sendCloud();

  void cloudPublisher();

  void cloudViewer();

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;

  void createLookup(size_t width, size_t height);

};

#endif // KINECT2_RECEIVER_H
