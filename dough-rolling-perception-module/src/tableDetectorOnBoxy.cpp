/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * tableDetectorOnBoxy.cpp
 *
 * Created on : Nov 18, 2014
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
// PCL specific includes
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
#include <pcl/filters/statistical_outlier_removal.h>
// TF Stuff
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
// OPENCV Stuff
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//Dough Pose Service
#include <lasa_perception_module/doughPose.h>
//Eigen Stuff
#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace cv;
using namespace std;

// Initialize ROS
ros::Publisher pub;
ros::ServiceClient client;
image_transport::Publisher pubIm;

vector<int> dough_center;
vector<int> dough_D1;
vector<int> dough_D2;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
	    
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
      // Convert to the templated PointCloud
      pcl::MsgFieldMap field_map;
      pcl::createMapping<pcl::PointXYZRGB> (cloud->fields, field_map);    
      pcl::fromPCLPointCloud2 (*cloud, *cloud_blob, field_map);
      
      cloud_filtered=cloud_blob;
//       Check for NaNs
      for (size_t y = 0; y < cloud_blob->height; ++y){
		  for (size_t x = 0; x < cloud_blob->width; ++x){
		     pcl::PointXYZRGB p = cloud_blob->at(x, y);
		      if (isnan(p.x) || isnan(p.y) || isnan(p.z)){
			  p.x = 0;
		          p.y = 0;
			  p.z = 0;
		      }		      
		      if (p.z > 1.5){
			  p.x = 0;
		          p.y = 0;
			  p.z = 0;
		      }
		      if (p.x > 0.5 || p.x < -0.5) {
			  p.x = 0;
		          p.y = 0;
			  p.z = 0;
		      }			      
		      cloud_filtered->at(x, y) = p;
		  }
      }
      
      
      // Create the filtering objects 
      pcl::PassThrough<pcl::PointXYZRGB> pass;

      cloud_filtered=cloud_blob;
      cout << cloud_filtered->height << "x" << cloud_filtered->width << std::endl;      

      
      // Create the segmentation object
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());     
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.1);
      
      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      
      int i=0, nr_points = (int) cloud_filtered->points.size ();
      pcl::IndicesPtr remaining (new std::vector<int>);
      remaining->resize (nr_points);
      for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }
  
      // Segment the largest planar component from the cloud
      seg.setInputCloud (cloud_filtered);
      seg.setIndices (remaining);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
	ROS_ERROR( "Could not estimate a planar model for the given dataset.");
      }

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.setKeepOrganized (true);
      extract.filter (*cloud_p);
      
      //Get Bounding Box and apply to original cloud
      Eigen::Vector4f min_point; 
      Eigen::Vector4f max_point; 
      pcl::getMinMax3D (*cloud_p, min_point, max_point); 
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (min_point(0), max_point(0));
//       pass.setFilterLimits (-0.5, 0.5);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (min_point(1), max_point(1));
      pass.setFilterFieldName ("z");
//       pass.setFilterLimits (min_point(2)-0.1, max_point(2));      
      pass.setFilterLimits (0.7, 1.5);  
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      // Convert to ROS data type
      pcl::PCLPointCloud2 output;
      pcl::toPCLPointCloud2(*cloud_filtered,output);
      
      // Publish the table point cloud
      pub.publish (output);
      
      //Create image and SEND TO Dough Detector
      Mat rgb_im (cloud_filtered->height, cloud_filtered->width, CV_8UC3);
      for (size_t y = 0; y < cloud_filtered->height; ++y){
		  for (size_t x = 0; x < cloud_filtered->width; ++x){
		    pcl::PointXYZRGB p = cloud_filtered->at(x, y);
		    Eigen::Vector3i rgb = p.getRGBVector3i(); 
		      rgb_im.at<unsigned char>(y, x*3 + 0) = rgb(2);
                      rgb_im.at<unsigned char>(y, x*3 + 1) = rgb(1);
                      rgb_im.at<unsigned char>(y, x*3 + 2) = rgb(0);
	}  
      }
      
      ROS_INFO( "Sending Segmented Image to Dough Detector");
      //Convert to image message
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_im).toImageMsg(); 
      lasa_perception_module::doughPose srv;
      srv.request.Table_image = *msg;
      if (client.call(srv)){
	  ROS_INFO("Sending Segmented Image to Dough Detector");
	  ROS_INFO_STREAM("Dough Center: [" << srv.response.dough_center[0] << "," << srv.response.dough_center[1]<<"]");
	  ROS_INFO_STREAM("Dough Direction 1: [" << srv.response.dough_D1[0] << "," << srv.response.dough_D1[1]<<"]");
	
      }
      else{
	  ROS_ERROR("Failed to call service dough_Pose");
      }      
      
      ROS_INFO( "received dough pose");
      Vector<int> dough_Center (srv.response.dough_center);
      Vector<int> dough_D1 (srv.response.dough_D1);
      
      ROS_INFO( "converting to point cloud");
      pcl::PointXYZRGB dough = cloud_blob->at(dough_Center[0], dough_Center[1]);
      pcl::PointXYZRGB dough_pc = cloud_blob->at(dough_D1[0], dough_D1[1]);      
      
      // Create and Publish Table TF
	static tf::TransformBroadcaster br;      
	tf::Vector3 t(dough.x,dough.y,dough.z);
	tf::Transform dough_transform, dough_transform1;

      
	ROS_INFO_STREAM("Dough Position: " << dough.x <<" "<< dough.y<<" "<<dough.z); 
	ROS_INFO_STREAM("Dough D1 Position: " << dough_pc.x <<" "<< dough_pc.y<<" "<<dough_pc.z);
	
	
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	tf::StampedTransform base2kinect;
	try{
	   
	     listener.waitForTransform("head_xtion_rgb_optical_frame","base_link", ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform("head_xtion_rgb_optical_frame","base_link", ros::Time(0), base2kinect);
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
	tf::Vector3 t1(dough_pc.x,dough_pc.y,dough_pc.z);
	dough_transform1.setOrigin(t1);
	
	tf::Matrix3x3 tf3d;
// 	tf3d.setValue(static_cast<double>(R(0,0)), static_cast<double>(R(0,1)), static_cast<double>(R(0,2)), 
// 	    static_cast<double>(R(1,0)), static_cast<double>(R(1,1)), static_cast<double>(R(1,2)), 
// 	    static_cast<double>(R(2,0)), static_cast<double>(R(2,1)), static_cast<double>(R(2,2)));
	tf::matrixEigenToTF(R,tf3d);
		
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
      
	dough_transform.setOrigin(t);
	dough_transform.setRotation(tfqt);
	dough_transform1.setRotation(tfqt);
	
// 	Eigen::Vector3d zdir(-coefficients->values[0],-coefficients->values[1],-coefficients->values[2]);
	Eigen::Vector3d zdir (R(0,2),R(1,2),R(2,2));
	Eigen::Vector3d xdir (-dough_pc.x + dough.x, -dough_pc.y+dough.y , -dough_pc.z+dough.z);
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
	br.sendTransform(tf::StampedTransform(dough_transform, ros::Time::now(), "/head_xtion_rgb_optical_frame", "/dough_frame"));
	br.sendTransform(tf::StampedTransform(dough_transform1, ros::Time::now(), "/head_xtion_rgb_optical_frame", "/dough_D1"));
    }
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "table_Detector");
  ros::NodeHandle nh;
  
  //   Create a ROS Service Client for the dough pose
  client = nh.serviceClient<lasa_perception_module::doughPose>("dough_Pose");
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/head_xtion/depth_registered/points", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("/table", 1);
  
  // Spin
  ros::spin ();
}

//       pass.setInputCloud (cloud_blob);	
//       pass.setFilterFieldName ("z");
//       pass.setFilterLimits (0.0, 2);
//       pass.setFilterLimitsNegative (true);
//       pass.setFilterFieldName ("x");
//       pass.setFilterLimits (0.5, 0.5);
//       pass.setKeepOrganized (true);
//       pass.filter (*cloud_filtered);
      
/*      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (cloud_filtered);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.setKeepOrganized (true);
      sor.filter (*cloud_filtered); */     
// OLD HACKY STUFF
// 	tf::Matrix3x3 tf3d;
// 	tf3d.setValue(static_cast<double>(R(0,0)), static_cast<double>(R(0,1)), static_cast<double>(R(0,2)), 
// 	    static_cast<double>(R(1,0)), static_cast<double>(R(1,1)), static_cast<double>(R(1,2)), 
// 	    static_cast<double>(R(2,0)), static_cast<double>(R(2,1)), static_cast<double>(R(2,2)));
// 
// 	tf::Quaternion tfqt;
// 	tf3d.getRotation(tfqt);
//       
// 	dough_transform.setOrigin(t);
// 	dough_transform.setRotation(tfqt);
// 	dough_transform1.setRotation(tfqt);
//       imshow("Segmented Image Sent to Dough Detection", rgb_im);
//       waitKey(1);