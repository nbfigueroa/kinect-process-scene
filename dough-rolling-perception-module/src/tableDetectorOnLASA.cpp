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
#include <pcl/common/transforms.h>
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
#include <lasa_perception_module/attractorPose.h>
//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry> 
//-- Boost Stuff --//
#include<boost/thread.hpp>

using namespace cv;
using namespace std;

//-- Declare Parameters --//
string camera, output_pc, dough_pose_srv;
double depth_cutoff, plane_threshold;
string world_frame, dough_frame, dough_attractor;
string input_point_cloud;
string roller_direction, outward_direction, camera_away;
boost::mutex mutex;

//-- Initialize ROS --//
ros::Publisher pub;
ros::ServiceClient client;
image_transport::Publisher pubIm;
volatile bool service_flag, robot_ok=false, has_NaNs=false;

vector<int> dough_center;
vector<int> dough_D1;
vector<int> dough_D2;
double pixel_area, dough_area, radius_a, radius_b, pixel_m_conv (0.0027);
bool dough_found, use_base_for_z;

//Offset due to vision errors (Kinect Calibration RGB-IR)
double x_off(0.0), y_off(0.0);

tf::Transform dough_transform, attractor_transform;
tf::Transform reach_center_attractor_transform, reach_corner_attractor_transform, back_attractor_transform, roll_attractor_transform;
tf::Transform dough_, reach_center_, reach_corner_, back_, roll_;
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

	if(!n.getParam("outward_direction", outward_direction)) {
		ROS_ERROR("Must provide the robot/world outward direction!!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Outward Direction: "<<outward_direction);
	}

	if(!n.getParam("roller_direction", roller_direction)) {
		ROS_ERROR("Must provide the roller direction!!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Rolling Direction: "<<roller_direction);
	}

	if(!n.getParam("camera_away", camera_away)) {
		ROS_ERROR("Must provide the camera away direction!!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Camera Away Direction: "<<camera_away);
	}

	if(!n.getParam("x_offset", x_off)) {
		ROS_ERROR("Must provide the camera offset in x-direction wrt. world frame!!");
		ret = false;
	} else {
		ROS_INFO_STREAM("X Offset: "<<x_off);
	}

	if(!n.getParam("y_offset", y_off)) {
		ROS_ERROR("Must provide the camera offset in x-direction wrt. world frame!!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Y Offset: "<<y_off);
	}

	if(!n.getParam("use_base_for_z", use_base_for_z)) {
		ROS_ERROR("Must decide whether using base or plane for Z attractor orientation !!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Use Base for Z attractor Orientation: "<<use_base_for_z);
	}


	return ret;
}


bool
checkforNans(tf::Transform& pose){
    if(std::isnan(pose.getOrigin().x()))
	return 1;
    else if(std::isnan(pose.getOrigin().y())) 
	return 1;
    else if (std::isnan(pose.getOrigin().z()))
        return 1;
    else
	return 0;
}

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{

//     if(service_flag){
	ROS_INFO("Attractor Estimation Began...");
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_smooth (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_trans_filt (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      //-- Convert to the templated PointCloud --//
      pcl::MsgFieldMap field_map;
      pcl::createMapping<pcl::PointXYZRGB> (cloud->fields, field_map);    
      pcl::fromPCLPointCloud2 (*cloud, *cloud_blob, field_map);  
      
      //Transform Point Cloud to Base RF
      tf::TransformListener listener;
      ros::Rate rate(10.0);
      tf::StampedTransform base2kinect;
      string camera_frame = "/kinect2_rgb_optical_frame";
      try{	    
		    listener.waitForTransform(world_frame,camera_frame, ros::Time(0), ros::Duration(10.0) );
		    listener.lookupTransform(world_frame,camera_frame, ros::Time(0), base2kinect);
		    robot_ok= true;	
	      }
       catch (tf::TransformException ex){
		    robot_ok = false;	
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
	}	      
	
     if (robot_ok){
	
      //-> Use Base RF for Bounding Box
      tf::Transform b2k = base2kinect;
      tf::Transform kinect2base = base2kinect.inverse();
      Eigen::Affine3d k2b_eig, b2k_eig; 
      tf::transformTFToEigen(kinect2base,k2b_eig);
      tf::transformTFToEigen(b2k,b2k_eig);
      pcl::transformPointCloud (*cloud_blob, *cloud_transformed, b2k_eig);


      //-- Filter out unwanted scene --//

      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (cloud_transformed);	
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (0.15, 0.8);
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (-0.3, 0.7); 
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (-0.25,0.5);
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);

      pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, k2b_eig);
  
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    /*  // Filter using Kinect RF
      //-- Filter out unwanted scene --//
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (cloud_blob);	
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (-0.03, 1);
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("y");
//      pass.setFilterLimits (-depth_cutoff,0.5);
      pass.setFilterLimits (-0.2,0.5);
      //pass.setFilterLimits (-depth_cutoff,0); //LASA
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
      
      pass.setInputCloud (cloud_filtered);	
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.5, depth_cutoff); 
      pass.setKeepOrganized (true);
      pass.filter (*cloud_filtered);
  
*/
    
      //-- Smooth out data with Fast BilateralFilter --//
      // No need for Smoothing with Kinect 2
      /*pcl::FastBilateralFilter<pcl::PointXYZRGB> bFilter; 
      bFilter.setInputCloud(cloud_filtered); 
      bFilter.setSigmaR(0.05f); 
      bFilter.setSigmaS(3.0f); 
      bFilter.applyFilter(*cloud_filtered);
      

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      // NOT USING THIS ON BOXY AS WE KNOW EXACTLY WHERE THE TABLE WILL BE 
      //-- Create the segmentation object --//
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());     
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      
      // Optional
      seg.setOptimizeCoefficients (false);
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
      ROS_INFO_STREAM("Nr points:"<<nr_points);
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
      extract.filter (*cloud_plane);*/
      

      *cloud_plane = *cloud_filtered;
      //-- Get Bounding Box and apply to original cloud --//
      Eigen::Vector4f min_point; 
      Eigen::Vector4f max_point; 
      pcl::getMinMax3D (*cloud_plane, min_point, max_point);
      
      
       //-- Publish the table point cloud --//
      pcl::PCLPointCloud2 output;
      pcl::toPCLPointCloud2(*cloud_plane,output);
      pub.publish (output);
      

      //-- Create binary image and SEND TO Dough Detector Service --//
      Mat bw_im (cloud_plane->height, cloud_plane->width, CV_8UC1);
      for (size_t y = 0; y < cloud_plane->height; ++y){
		  for (size_t x = 0; x < cloud_plane->width; ++x){
		    pcl::PointXYZRGB p = cloud_plane->at(x, y);
		    if ((std::isnan(p.x) && std::isnan(p.y) && std::isnan(p.z)) != 1) {
			bw_im.at<unsigned char>(y, x) = 255;  
		    }	
		    else
		      bw_im.at<unsigned char>(y, x) = 0;  
		  }	
      }
      ROS_INFO_STREAM("Succesfully created bw image");
 
      //-- Query Dough Pose Service --//      
      ROS_INFO( "Querying Dough Pose Service");
      //-- Sending BW image --//
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", bw_im).toImageMsg(); 
      lasa_perception_module::doughPose srv;
      srv.request.Table_image = *msg;
      if (client.call(srv)){
	  ROS_INFO_STREAM("Dough Center: [" << srv.response.dough_center[0] << "," << srv.response.dough_center[1]<<"]");
	  ROS_INFO_STREAM("Dough Postive Direction: [" << srv.response.dough_D1[0] << "," << srv.response.dough_D1[1]<<"]");
	  ROS_INFO_STREAM("Dough Negative Direction: [" << srv.response.dough_D2[0] << "," << srv.response.dough_D2[1]<<"]");

	  if (!srv.response.dough_found){
	    ROS_ERROR("DOUGH! WAS NOT LOCATED!!");
	    dough_found = 0;
	  }
	  else{
	      dough_found = 1;
	      //-- Converting to 3D --//      
	      Vector<int> dough_Center (srv.response.dough_center);
	      Vector<int> dough_Attractor (srv.response.dough_D1);       
	      Vector<int> dough_Attractor_Neg (srv.response.dough_D2);    
	      pixel_area = srv.response.area;
	      radius_a = srv.response.radius_a;
	      radius_b = srv.response.radius_b;
	      pcl::PointXYZRGB dough = cloud_blob->at(dough_Center[0], dough_Center[1]);
	      pcl::PointXYZRGB dough_attr = cloud_blob->at(dough_Attractor[0], dough_Attractor[1]);      
	      pcl::PointXYZRGB dough_attr_neg = cloud_blob->at(dough_Attractor_Neg[0], dough_Attractor_Neg[1]);      

		
	      if(std::isnan(dough.x) || std::isnan(dough.y) || std::isnan(dough.z)){
			dough_found = 0;
		  	ROS_ERROR("DOUGH POSITION GIVES NANS!! Invalid 3D point, move robot maybe!");	
			return;
	      }
	      else
	         ROS_INFO_STREAM("Dough Position: " << dough.x <<" "<< dough.y<<" "<<dough.z); 


	      if(std::isnan(dough_attr.x) || std::isnan(dough_attr.y) || std::isnan(dough_attr.z)){
			dough_found = 0;
		  	ROS_ERROR("DOUGH POSITIVE ATTRACTOR GIVES NANS!! Invalid 3D point, move robot maybe!");	
			return;
	      }
		else
	      ROS_INFO_STREAM("Dough Positive Attractor Position: " << dough_attr.x <<" "<< dough_attr.y<<" "<<dough_attr.z);


	      if(std::isnan(dough_attr_neg.x) || std::isnan(dough_attr_neg.y) || std::isnan(dough_attr_neg.z)){
			dough_found = 0;
		  	ROS_ERROR("DOUGH NEGATIVE ATTRACTOR GIVES NANS!! Invalid 3D point, move robot maybe!");	
			return;
	      }
		else
	      ROS_INFO_STREAM("Dough Negative Attractor Position: " << dough_attr_neg.x <<" "<< dough_attr_neg.y<<" "<<dough_attr_neg.z);

	      ROS_INFO_STREAM("Dough Area in Pixels: " << pixel_area);
	      ROS_INFO_STREAM("Ellipse Radius a in Pixels: " << radius_a);
	      ROS_INFO_STREAM("Ellipse Radius b in Pixels: " << radius_b);

	      double ellipse_area_px = 3.14159*radius_a*radius_b;
	      double ellipse_area_m =  ellipse_area_px*pow(pixel_m_conv,2);

	      ROS_INFO_STREAM("Ellipse Area in Pixels: " << ellipse_area_px);
	      ROS_INFO_STREAM("Ellipse Area in m^2: " << ellipse_area_m);

	      //--Estimated ellipse are in m^2 will be the dough area --//
	      dough_area = ellipse_area_m;

	      
	      //------- Create Dough Frame Transform pointing to positive attractor------//
	      tf::Transform dough_transform_cam;
	      tf::Vector3 t(dough.x,dough.y,dough.z);      
	      dough_transform_cam.setOrigin(t);
	      tf::Matrix3x3 tf3d;
	      tf::Quaternion tfqt;
	      

	      // Define Z direction of attractors
	      Eigen::Vector3d zdir;
	      if (use_base_for_z){
	    	  //-> Use Base Orientation
	    	  tf::Transform kinect2base = base2kinect.inverse();
	    	  tf::Matrix3x3 R = kinect2base.getBasis();
	    	  zdir = Eigen::Vector3d(R[0][2],R[1][2],R[2][2]);
	      }
	      else
	    	  zdir = Eigen::Vector3d (-coefficients->values[0],-coefficients->values[1],-coefficients->values[2]);

	      Eigen::Vector3d xdir (dough_attr.x - dough.x, dough_attr.y - dough.y , dough_attr.z - dough.z);	     	      
	      Eigen::Vector3d Rx = xdir/xdir.norm();
	      Eigen::Vector3d Rz = zdir/zdir.norm();
	      Eigen::Vector3d Ry = Rz.cross(Rx)/(Rz.cross(Rx)).norm();
	      
	      Eigen::Matrix3d Rnew;
	      Rnew << Rx, Ry, Rz;
	      tf::matrixEigenToTF(Rnew,tf3d);
	      tf3d.getRotation(tfqt);
	      dough_transform_cam.setRotation(tfqt);
	      dough_transform = base2kinect*dough_transform_cam;
	      
	      //------- Create Candidate Attractor Frame Transforms -------//
	      //-> Positive attractor transform
	      tf::Transform pos_attractor_transform, pos_attractor_transform_cam;
	      tf::Vector3 t1 (dough_attr.x,dough_attr.y,dough_attr.z);
	      pos_attractor_transform_cam = dough_transform_cam;
	      pos_attractor_transform_cam.setOrigin(t1);	      
	      pos_attractor_transform = dough_transform.inverse()*base2kinect*pos_attractor_transform_cam;
	      
	      //-> Negative attractor transform
	      tf::Vector3 t_neg(dough_attr_neg.x,dough_attr_neg.y,dough_attr_neg.z);	      
	      tf::Transform neg_attractor_transform, neg_attractor_transform_cam;      
	      neg_attractor_transform_cam = pos_attractor_transform_cam;
	      neg_attractor_transform_cam.setOrigin(t_neg);
	      neg_attractor_transform = dough_transform.inverse()*base2kinect*neg_attractor_transform_cam;
	  		      
	      //-> attractor correction
	      xdir = xdir - (xdir.dot(zdir))*zdir;
	      Rx = xdir/xdir.norm();
	      Ry = Rz.cross(Rx)/(Rz.cross(Rx)).norm();
	      Rnew << Rx, Ry, Rz;
	      tf::matrixEigenToTF(Rnew,tf3d);
	      tf3d.getRotation(tfqt);
	      dough_transform_cam.setRotation(tfqt);
	      dough_transform = base2kinect*dough_transform_cam;
	      pos_attractor_transform = dough_transform*pos_attractor_transform;
	      neg_attractor_transform = dough_transform*neg_attractor_transform;
	     
	      
	      //-> Check which attractor is farther away from outward world direction
	      tf::Vector3 center(dough_transform.getOrigin());
	      tf::Vector3 positive(pos_attractor_transform.getOrigin());
	      tf::Vector3 negative(neg_attractor_transform.getOrigin());

	      tf::Vector3 positive_dir (positive-center);
	      tf::Vector3 negative_dir (negative-center);

	      tf::Vector3 out_dir;
	      if(outward_direction=="-y"){
	    	  out_dir = tf::Vector3(0,-1,0);
	      }
	      if(outward_direction=="x"){
	    	  out_dir = tf::Vector3(1,0,0);
	      }
	      
	      if(outward_direction=="-x"){
	      		out_dir = tf::Vector3(-1,0,0);
	      }

	      double pos_dir_scalar = out_dir.dot(positive_dir);
	      double neg_dir_scalar = out_dir.dot(negative_dir);
	      ROS_INFO_STREAM("Dot Product of outward direction with positive attractor: "<<pos_dir_scalar);
	      ROS_INFO_STREAM("Dot Product of outward direction with negative attractor: "<<neg_dir_scalar);
			  
	      //-> Choose attractors for a roll sequence 
	      Eigen::Matrix3d Rot_roll_dir;
	      Rot_roll_dir << -1, 0, 0,
			      0, -1, 0,
			      0, 0 ,1;      
		      
	      //-> Roll Attractor
	      if (pos_dir_scalar>0){
	    	  roll_attractor_transform = pos_attractor_transform;
	    	  reach_corner_attractor_transform = neg_attractor_transform;
	      }else{
	    	  roll_attractor_transform = neg_attractor_transform;
	    	  reach_corner_attractor_transform = pos_attractor_transform;

	    	  tf::Matrix3x3 att_r = roll_attractor_transform.getBasis();
	    	  Eigen::Matrix3d r;
	    	  tf::matrixTFToEigen(att_r,r);
	    	  Rot_roll_dir = Rot_roll_dir*r;
	    	  tf::matrixEigenToTF(Rot_roll_dir,tf3d);
	    	  tf3d.getRotation(tfqt);
	    	  roll_attractor_transform.setRotation(tfqt);
	    	  reach_corner_attractor_transform.setRotation(tfqt);
	    	  dough_transform.setRotation(tfqt);
	      }
	      //-> Reach Center Attractor
	      reach_center_attractor_transform = dough_transform;

	      //-> Back Attractor
	      back_attractor_transform = reach_center_attractor_transform;
	      tf::Vector3 t_b	(back_attractor_transform.getOrigin());

	      if(camera_away=="y"){
		     back_attractor_transform.setOrigin(tf::Vector3(t_b[0],t_b[1]+0.4,t_b[2]+0.25));
	      }

	      if(camera_away=="-y"){
		     back_attractor_transform.setOrigin(tf::Vector3(t_b[0]-0.15,t_b[1]-0.2,t_b[2]+0.15));
	      }
		      
	      //-> Orient attractors based on Tool Configuration      			  
	      tf::Matrix3x3 att_r = roll_attractor_transform.getBasis();
	      tf::Vector3 y_dir;
	      if (roller_direction=="-y"){
	    	  y_dir = tf::Vector3(-att_r.getColumn(0));
	      }
	      else{
	    	  y_dir = tf::Vector3(att_r.getColumn(1));
	      }
	      tf::Vector3 z_dir (-att_r.getColumn(2));
	      tf::Vector3 x_dir = y_dir.cross(z_dir);
	      
	      tf3d[0][0] = x_dir[0];
	      tf3d[1][0] = x_dir[1];
	      tf3d[2][0] = x_dir[2];
	      
	      tf3d[0][1] = y_dir[0];
	      tf3d[1][1] = y_dir[1];
	      tf3d[2][1] = y_dir[2];      
	      
	      tf3d[0][2] = z_dir[0];
	      tf3d[1][2] = z_dir[1];
	      tf3d[2][2] = z_dir[2];
	      
	      tf3d.getRotation(tfqt);
	      reach_center_attractor_transform.setRotation(tfqt);
	      reach_corner_attractor_transform.setRotation(tfqt);
	      roll_attractor_transform.setRotation(tfqt);
	      back_attractor_transform.setRotation(tfqt);
	      

	     if (checkforNans(reach_center_attractor_transform) || checkforNans(reach_corner_attractor_transform) || checkforNans(roll_attractor_transform) || checkforNans(back_attractor_transform))
		has_NaNs = 1;
	     else
		has_NaNs = 0;
		   
		      //---------- MUTEX ---------//
		      {
			mutex.lock();
		
			dough_ =  dough_transform;
			reach_center_ = reach_center_attractor_transform;
			reach_corner_ = reach_corner_attractor_transform;
			roll_ = roll_attractor_transform;
			back_ = back_attractor_transform;
		      
			//-> Offset due to calibration
			tf::Vector3 t_off	(dough_.getOrigin());
			dough_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2]));
			t_off = 	tf::Vector3(reach_center_.getOrigin());
			//reach_center_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2]));
			reach_center_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2]));
			t_off = 	tf::Vector3(reach_corner_.getOrigin());
			reach_corner_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2]));
			t_off = 	tf::Vector3(roll_.getOrigin());
			roll_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2]));
			t_off = 	tf::Vector3(back_.getOrigin());
			back_.setOrigin(tf::Vector3(t_off[0]+x_off,t_off[1]+y_off,t_off[2])); 

			//-> Transform Attractor relative to dough frame     			  
			reach_center_ = dough_transform.inverse()*reach_center_;
			reach_corner_ = dough_transform.inverse()*reach_corner_;
			roll_ = dough_transform.inverse()*roll_;
			back_ = dough_transform.inverse()*back_;

			//-> Add Roller offset to reach corner	      
			tf::Vector3 roller_off (reach_corner_.getOrigin());
			reach_corner_.setOrigin(tf::Vector3(roller_off[0]+0.03,roller_off[1],roller_off[2]-0.02));	
			
			tf::Vector3 roller_c_off (reach_center_.getOrigin());
			reach_center_.setOrigin(tf::Vector3(roller_c_off[0],roller_c_off[1],roller_c_off[2]-0.02));	


			//-> Add little offset to roll attractor	      
			tf::Vector3 roll_att_off (roll_.getOrigin());
			roll_.setOrigin(tf::Vector3(roll_att_off[0]-0.01,roll_att_off[1],roll_att_off[2]-0.01));	


			mutex.unlock();
		      }
		      
		      //-- Publish transforms to TF --//
		      static tf::TransformBroadcaster br;
		      if (std::isnan(dough.x) || std::isnan(dough.y) || std::isnan(dough.z)){
			ROS_INFO_STREAM("Some nan in rotation" << Rx[0]);
			ROS_ERROR("NANs in estimated dough position");
			dough_found = 0;
			return;
		      }
		      else{
			ROS_INFO( "Sending TFssss");	
			br.sendTransform(tf::StampedTransform(dough_transform, ros::Time::now(), world_frame, dough_frame));

			br.sendTransform(tf::StampedTransform(reach_center_attractor_transform, ros::Time::now(), world_frame, "/reach_center"));
			br.sendTransform(tf::StampedTransform(reach_corner_attractor_transform, ros::Time::now(), world_frame, "/reach_corner"));
			br.sendTransform(tf::StampedTransform(roll_attractor_transform, ros::Time::now(), world_frame, "/roll"));
			br.sendTransform(tf::StampedTransform(back_attractor_transform, ros::Time::now(), world_frame, "/back"));
		    }  
	  
	}  
	  
      }
      else{
	  ROS_ERROR("Failed to call service dough_Pose");
      }      
      }

}

bool attractor_pose(lasa_perception_module::attractorPose::Request  &req, lasa_perception_module::attractorPose::Response &res){      
  
      ROS_INFO_STREAM("Request attractors for dough rolling sequence:" << req.detect);
      
      ROS_INFO_STREAM("Waiting for dough detection/attractor pose estimation callback...");
       
      //---------- MUTEX ---------//
      {
      mutex.lock();
       
      geometry_msgs::Transform dough_msg, reach_center_msg, reach_corner_msg, roll_msg, back_msg;            
      tf::transformTFToMsg(dough_,dough_msg);	
      tf::transformTFToMsg(reach_center_,reach_center_msg);
      tf::transformTFToMsg(reach_corner_,reach_corner_msg);
      tf::transformTFToMsg(roll_,roll_msg);
      tf::transformTFToMsg(back_,back_msg);
        
      
      res.dough_found = dough_found;
      if (dough_found){
	res.object_frame = dough_msg;
	res.reach_center_attractor = reach_center_msg;
	res.reach_corner_attractor = reach_corner_msg;
	res.roll_attractor = roll_msg;
	res.back_attractor = back_msg;
	res.area = dough_area;	
      }
     
      mutex.unlock();
      }     
      
      return true;
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
  
  //-- Create a ROS publisher for the output point cloud --//
  pub = nh.advertise<pcl::PCLPointCloud2> (output_pc, 1);
 
  //-- Create a ROS subscriber for the input point cloud --//
  //input_point_cloud = camera +  "/depth_registered/points";
  // For Kinect 2
  input_point_cloud = camera +  "/depth_lowres/points";
  ROS_INFO_STREAM("Input Point Cloud Topic Name: "<<input_point_cloud);
  ros::Subscriber sub = nh.subscribe (input_point_cloud, 1, cloud_cb);

  service_flag = false;
  //-- Create Service Server for Attractor Pose Estimation --//
  ros::ServiceServer service = nh.advertiseService("attractor_pose", attractor_pose);
 
  ros::AsyncSpinner spinner(2);
//   spinner.spin();
  spinner.start();
  ros::waitForShutdown();
}




/*      geometry_msgs::Transform dough_msg, attractor_msg;            
      tf::transformTFToMsg(dough_transform,dough_msg);	
      tf::transformTFToMsg(attractor_transform,attractor_msg);
      res.object_frame = dough_msg;
      res.attractor = attractor_msg;*/  

// if (action_phase=="reach_corner"){
// 	if (pos_dir_scalar<0){
// 	  attractor_transform = pos_attractor_transform;
// 	   tf::Matrix3x3 att_r = attractor_transform.getBasis();
// 	   Eigen::Matrix3d r;
// 	   tf::matrixTFToEigen(att_r,r);
// 	   Rot_roll_dir = Rot_roll_dir*r;
// 	   tf::matrixEigenToTF(Rot_roll_dir,tf3d);
// 	   tf3d.getRotation(tfqt);
// 	   attractor_transform.setRotation(tfqt);
// 	   dough_transform.setRotation(tfqt);
// 	   
// 	}else if(pos_dir_scalar>0 && neg_dir_scalar>0){
// 	   if(pos_dir_scalar < neg_dir_scalar)
// 	   {
// 	     attractor_transform = pos_attractor_transform;	     
// 	   tf::Matrix3x3 att_r = attractor_transform.getBasis();
// 	   Eigen::Matrix3d r;
// 	   tf::matrixTFToEigen(att_r,r);
// 	   Rot_roll_dir = Rot_roll_dir*r;
// 	   tf::matrixEigenToTF(Rot_roll_dir,tf3d);
// 	   tf3d.getRotation(tfqt);
// 	   attractor_transform.setRotation(tfqt);
// 	   dough_transform.setRotation(tfqt);
// 	  }else{
// 	     attractor_transform = neg_attractor_transform;
// 	  }
// 	}	
// 	else{
// 	  attractor_transform = neg_attractor_transform;
// 	}  
//       }
//       else{
// 	if (pos_dir_scalar>0){
// 	  attractor_transform = pos_attractor_transform;
// 	}else{
// 	  attractor_transform = neg_attractor_transform;
// 	   tf::Matrix3x3 att_r = attractor_transform.getBasis();
// 	   Eigen::Matrix3d r;
// 	   tf::matrixTFToEigen(att_r,r);
// 	   Rot_roll_dir = Rot_roll_dir*r;
// 	   tf::matrixEigenToTF(Rot_roll_dir,tf3d);
// 	   tf3d.getRotation(tfqt);
// 	   attractor_transform.setRotation(tfqt);
// 	   dough_transform.setRotation(tfqt);
// 	}  
// 	
// 	if (action_phase=="reach_center"){
// 	  attractor_transform = dough_transform;
// 	} 
// 	if(action_phase=="back"){
// 	   attractor_transform = dough_transform;
// 	   tf::Vector3 t (attractor_transform.getOrigin());
// 	   attractor_transform.setOrigin(tf::Vector3(t[0],t[1]+0.3,t[2]+0.25));
// 	}  	
//       }










      //-- Sending rgb image --//
//       sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_im).toImageMsg(); 

//       Eigen::Matrix3d R;
//       tf::Matrix3x3 tf3d;
//       tf::matrixEigenToTF(R,tf3d);
// 	      
//       tf::Quaternion tfqt;
//       tf3d.getRotation(tfqt);      
//       dough_transform.setRotation(tfqt);
//       attractor_transform.setRotation(tfqt);

//       //-- Create rgb image and SEND TO Dough Detector Service --//
//       Mat rgb_im (cloud_plane->height, cloud_plane->width, CV_8UC3);
//       for (size_t y = 0; y < cloud_plane->height; ++y){
// 		  for (size_t x = 0; x < cloud_plane->width; ++x){
// 		    pcl::PointXYZRGB p = cloud_plane->at(x, y);
// 		    Eigen::Vector3i rgb = p.getRGBVector3i(); 
// 		      rgb_im.at<unsigned char>(y, x*3 + 0) = rgb(2);
//                       rgb_im.at<unsigned char>(y, x*3 + 1) = rgb(1);
//                       rgb_im.at<unsigned char>(y, x*3 + 2) = rgb(0);
// 		      
// 	}  
//       }
//      
//       ROS_INFO_STREAM("Succesfully created rgb image");
//       static tf::TransformBroadcaster br;      
//       tf::TransformListener listener;
//       ros::Rate rate(10.0);
//       tf::StampedTransform base2kinect;
//       string camera_frame = camera + "_rgb_optical_frame";
//       try{	    
// 	     listener.waitForTransform(camera_frame, world_frame, ros::Time(0), ros::Duration(10.0) );
// 	     listener.lookupTransform(camera_frame, world_frame, ros::Time(0), base2kinect);
//       }
//       catch (tf::TransformException ex){
// 	    ROS_ERROR("%s",ex.what());
// 	    ros::Duration(1.0).sleep();
//       }
//         
//       tf::Quaternion b2k_orient (base2kinect.getRotation());
//       Eigen::Quaterniond q;
//       tf::quaternionTFToEigen (b2k_orient, q);
//       Eigen::Matrix3d R(q);
//       cout << R << endl;
//       if (std::isnan(dough.x) || std::isnan(dough.y) || std::isnan(dough.z)){
// 	ROS_INFO_STREAM("Some nan in rotation" << Rx[0]);
// 	ROS_ERROR("NANs in estimated dough position");
//       }
//       else{
// 	ROS_INFO( "Sending TFssss");	
// 	br.sendTransform(tf::StampedTransform(dough_transform, ros::Time::now(), camera_frame, dough_frame));
// 	br.sendTransform(tf::StampedTransform(attractor_transform, ros::Time::now(), camera_frame, dough_attractor));
//     }
