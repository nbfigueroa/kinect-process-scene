
/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * tableDetectorOnLWR.cpp
 *
 * Created on : Nov 29, 2014
 * Author     : nbfigueroa
 * Email      : nadia.figueroafernandez@epfl.ch
 * Website    : lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <ros/ros.h>
//-- PCL specific includes --//
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/PointIndices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/fast_bilateral.h>
//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
//-- OPENCV Stuff --//
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//-- Dough Pose Service --//
#include <lasa_perception_module/doughPose.h>
//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace cv;
using namespace std;

//-- Declare Parameters --//
string camera, output_pc, dough_pose_srv;
double depth_cutoff, plane_threshold;
string world_frame, dough_frame, dough_attractor;

//-- Initialize ROS --//
ros::Publisher pub;
ros::ServiceClient client;
image_transport::Publisher pubIm;

vector<int> dough_center;
vector<int> dough_D1;
vector<int> dough_D2;

bool parseParams(const ros::NodeHandle& n) {


	bool ret = true;
	if(!n.getParam("camera_name", camera)) {
		ROS_ERROR("Must provide camera name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Camera name: "<<camera);
	}

	if(!n.getParam("world_frame_name", world_frame)) {
		ROS_ERROR("Must provide world frame name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("World frame name: "<<world_frame);
	}

	if(!n.getParam("depth_cutoff", depth_cutoff)) {
		ROS_ERROR("Must provide depth cutoff!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Depth Cut-off: "<<depth_cutoff);
	}
	
	if(!n.getParam("plane_threshold", plane_threshold)) {
		ROS_ERROR("Must provide depth cutoff!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Plane Model Threshold: "<<plane_threshold);
	}

	if(!n.getParam("output_point_cloud", output_pc)) {
		ROS_ERROR("Must provide output point cloud name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Output Point Cloud name: "<<output_pc);
	}
	
       if(!n.getParam("dough_pose_srv_name", dough_pose_srv)) {
		ROS_ERROR("Must provide dough pose service name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Dough Pose Service name: "<<dough_pose_srv);
	}
	
	if(!n.getParam("dough_frame", dough_frame)) {
		ROS_ERROR("Must provide desired dough frame name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Dough frame name: "<<dough_frame);
	}

	if(!n.getParam("dough_attractor", dough_attractor)) {
		ROS_ERROR("Must provide desired dough principal direction frame name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Dough Attractor frame name: "<<dough_attractor);
	}

	
	return ret;
}


void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
	    
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_smooth (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      //-- Convert to the templated PointCloud --//
      pcl::MsgFieldMap field_map;
      pcl::createMapping<pcl::PointXYZRGB> (cloud->fields, field_map);    
      pcl::fromPCLPointCloud2 (*cloud, *cloud_blob, field_map);  
      
      //-- Filter out unwanted scene --//
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (cloud_blob);	
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (-0.3, 0.3);
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.2, depth_cutoff); 
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      //-- Smooth out data with Fast BilateralFilter --//
      pcl::FastBilateralFilter<pcl::PointXYZRGB> bFilter; 
      bFilter.setInputCloud(cloud_filtered); 
      bFilter.setSigmaR(0.01f); 
      bFilter.setSigmaS(3.0f); 
      bFilter.applyFilter(*cloud_filtered); 
      
      //-- Publish the filtered/smoothed point cloud --//
      pcl::PCLPointCloud2 output;
      pcl::toPCLPointCloud2(*cloud_filtered,output);
      pub.publish (output);
      
      //-- Create the segmentation object --//
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());     
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (plane_threshold);
      
      //-- Create the filtering object --//
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      int i=0, nr_points = (int) cloud_filtered->points.size ();
      pcl::IndicesPtr remaining (new std::vector<int>);
      remaining->resize (nr_points);
      for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }
  
      //-- Segment the largest planar component from the cloud --//
      seg.setInputCloud (cloud_filtered);
      seg.setIndices (remaining);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
	ROS_ERROR( "Could not estimate a planar model for the given dataset.");
      }

      //-- Extract the inliers --//
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.setKeepOrganized (true);
      extract.filter (*cloud_plane);
      
      //-- Get Bounding Box and apply to original cloud --//
      Eigen::Vector4f min_point; 
      Eigen::Vector4f max_point; 
      pcl::getMinMax3D (*cloud_plane, min_point, max_point);
      
      
//       //-- Publish the table point cloud --//
//       pcl::PCLPointCloud2 output;
//       pcl::toPCLPointCloud2(*cloud_plane,output);
//       pub.publish (output);
      
      //-- Create image and SEND TO Dough Detector Service --//
      Mat rgb_im (cloud_plane->height, cloud_plane->width, CV_8UC3);
      for (size_t y = 0; y < cloud_plane->height; ++y){
		  for (size_t x = 0; x < cloud_plane->width; ++x){
		    pcl::PointXYZRGB p = cloud_plane->at(x, y);
		    Eigen::Vector3i rgb = p.getRGBVector3i(); 
		      rgb_im.at<unsigned char>(y, x*3 + 0) = rgb(2);
                      rgb_im.at<unsigned char>(y, x*3 + 1) = rgb(1);
                      rgb_im.at<unsigned char>(y, x*3 + 2) = rgb(0);
	}  
      }
      
      //-- Query Dough Pose Service --//      
      ROS_INFO( "Querying Dough Pose Service");
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_im).toImageMsg(); 
      lasa_perception_module::doughPose srv;
      srv.request.Table_image = *msg;
      if (client.call(srv)){
	  ROS_INFO_STREAM("Dough Center: [" << srv.response.dough_center[0] << "," << srv.response.dough_center[1]<<"]");
	  ROS_INFO_STREAM("Dough Direction 1: [" << srv.response.dough_D1[0] << "," << srv.response.dough_D1[1]<<"]");
	
      }
      else{
	  ROS_ERROR("Failed to call service dough_Pose");
      }      
      
      //-- Converting to 3D --//      
      Vector<int> dough_Center (srv.response.dough_center);
      Vector<int> dough_Attractor (srv.response.dough_D2);       
      pcl::PointXYZRGB dough = cloud_blob->at(dough_Center[0], dough_Center[1]);
      pcl::PointXYZRGB dough_attr = cloud_blob->at(dough_Attractor[0], dough_Attractor[1]);      
      ROS_INFO_STREAM("Dough Position: " << dough.x <<" "<< dough.y<<" "<<dough.z); 
      ROS_INFO_STREAM("Dough Attractor Position: " << dough_attr.x <<" "<< dough_attr.y<<" "<<dough_attr.z);
      
      //-- Create and Publish Dough Frame TF --//
      static tf::TransformBroadcaster br;      
      tf::Vector3 t(dough.x,dough.y,dough.z);
      tf::Transform dough_transform, dough_transform1;
      tf::TransformListener listener;
      ros::Rate rate(10.0);
      tf::StampedTransform base2kinect;
      string camera_frame = camera + "_rgb_optical_frame";
      try{	    
	     listener.waitForTransform(camera_frame, world_frame, ros::Time(0), ros::Duration(10.0) );
	     listener.lookupTransform(camera_frame, world_frame, ros::Time(0), base2kinect);
      }
      catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
      }
        
      tf::Quaternion b2k_orient (base2kinect.getRotation());
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen (b2k_orient, q);
      Eigen::Matrix3d R(q);
      cout << R << endl;
      
      dough_transform1.setIdentity();
      tf::Vector3 t1(dough_attr.x,dough_attr.y,dough_attr.z);
      dough_transform1.setOrigin(t1);
      
      tf::Matrix3x3 tf3d;
      tf::matrixEigenToTF(R,tf3d);
	      
      tf::Quaternion tfqt;
      tf3d.getRotation(tfqt);
    
      dough_transform.setOrigin(t);
      dough_transform.setRotation(tfqt);
      dough_transform1.setRotation(tfqt);

      //-> Use Plane Model Orientation
      Eigen::Vector3d zdir(-coefficients->values[0],-coefficients->values[1],-coefficients->values[2]);
      //-> Use Base Orientation
//       Eigen::Vector3d zdir (R(0,2),R(1,2),R(2,2));
      Eigen::Vector3d xdir (-dough_attr.x + dough.x, -dough_attr.y+dough.y , -dough_attr.z+dough.z);
      Eigen::Vector3d Rx = xdir/xdir.norm();
      Eigen::Vector3d Rz = zdir/zdir.norm();
      Eigen::Vector3d Ry = Rz.cross(Rx)/(Rz.cross(Rx)).norm();
      
      Eigen::Matrix3d Rnew;
      Rnew << Rx, Ry, Rz;
      tf::matrixEigenToTF(Rnew,tf3d);
      tf3d.getRotation(tfqt);
      dough_transform.setRotation(tfqt);
      dough_transform1.setRotation(tfqt);
	
      if (std::isnan(dough.x) || std::isnan(dough.y) || std::isnan(dough.z)){
	ROS_INFO_STREAM("Some nan in rotation" << Rx[0]);
	ROS_ERROR("NANs in estimated dough position");
      }
      else{
	ROS_INFO( "Sending TFssss");	
	br.sendTransform(tf::StampedTransform(dough_transform, ros::Time::now(), camera_frame, dough_frame));
	br.sendTransform(tf::StampedTransform(dough_transform1, ros::Time::now(), camera_frame, dough_attractor));
    }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "table_Detector");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  
  //-- Parse Parameters from Server --//
  if(!parseParams(_nh)) {
		ROS_ERROR("Errors while parsing arguments.");
		return 1;
  }
  
  //--  Create a ROS Service Client for the dough pose --//
  client = nh.serviceClient<lasa_perception_module::doughPose>(dough_pose_srv);
  
  //-- Create a ROS subscriber for the input point cloud --//
//  string input_point_cloud = camera +  "/depth_registered/points";
  // Kinect2 on Boxy
  string input_point_cloud = camera +  "/depth_lowres/points";
  ROS_INFO_STREAM("Input Point Cloud Topic Name: "<<input_point_cloud);
  ros::Subscriber sub = nh.subscribe (input_point_cloud, 1, cloud_cb);
  
  //-- Create a ROS publisher for the output point cloud --//
  pub = nh.advertise<pcl::PCLPointCloud2> (output_pc, 1);
  
  ros::spin ();
}
