
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
using namespace std;


//-- Declare Parameters --//
string input_cloud, output_cloud;
int num_objects;


//-- Parameter Reader--//
bool parseParams(const ros::NodeHandle& n) {
    bool ret = true;
    if(!n.getParam("input_cloud", input_cloud)) {
        ROS_ERROR("Must provide input cloud name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Input Cloud Name: "<< input_cloud);
    }

    if(!n.getParam("output_cloud", output_cloud)) {
        ROS_ERROR("Must provide output cloud name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Output Cloud Name: "<< output_cloud);
    }

    if(!n.getParam("num_objects", num_objects)) {
        ROS_ERROR("Must provide number of expected objects!");
        ret = false;
    } else {
        ROS_INFO_STREAM("num_objects: "<< num_objects);
    }
}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  //-- Convert to the templated PointCloud --//
  pcl::PointCloud<pcl::PointXYZRGB> cloudxx;
  // Convert ros msg cloud to pcl cloud
  pcl::fromROSMsg(*msg,cloudxx);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(cloudxx));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  //-- Remove background with pass-through filter --//
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input_cloud_ptr);
  pass.setKeepOrganized(true);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter (*filtered_cloud_ptr);

  pass.setInputCloud(filtered_cloud_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter (*filtered_cloud_ptr);

  pass.setInputCloud(filtered_cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter (*filtered_cloud_ptr);

  //-- Publish the table point cloud --//
  pcl::PCLPointCloud2 output;
  pcl::toPCLPointCloud2(*filtered_cloud_ptr, output);
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_tabletop");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  //-- Parse Parameters from Server --//
  if(!parseParams(_nh)) {
        ROS_ERROR("Errors while parsing arguments.");
        return 1;
  }

  //-- Create a ROS subscriber for the input point cloud --//
  ros::Subscriber sub = nh.subscribe (input_cloud, 1, cloud_cb);

  //-- Create a ROS publisher for the output point cloud --//
  pub = nh.advertise<sensor_msgs::PointCloud2> (output_cloud, 1);

  // Spin
  ros::spin ();
}
