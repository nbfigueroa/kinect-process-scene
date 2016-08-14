#include <ros/ros.h>
// Common
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// Filtering
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>


// Visualization Shit
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// TF Stuff
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>

ros::Publisher pub;
using namespace std;


//-- Declare Input/Output Cloud Names --//
string input_cloud, output_cloud, camera_frame;

//-- Declare PassFilter Parameters --//
double x_min(-1), x_max(1), y_min(-1), y_max(1), z_min(1), z_max(1.6);

//-- Declare StatOutliers Parameters --//
double stat_mean(10) , stat_std(1);

//-- Declare Plane Estimation Parameters --//
double norm_rad(0.03) ; //[m]


// Instantiate Cloud Viewer
bool pcl_viz(0);
boost::shared_ptr<pcl::visualization::PCLVisualizer> visor;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add properties-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.2);
  return viewer;
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    viewer->spinOnce();
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_,
               pcl::PointCloud<pcl::Normal>::ConstPtr normals_){
    viewer->removePointCloud("normals",0);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_, normals_, 50, 0.02, "normals");
    viewer->spinOnce();
}


void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb);
    viewer->spinOnce();
}


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

    if(!n.getParam("camera_frame_name", camera_frame)) {
        ROS_ERROR("Must provide camera frame name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Camera Frame Name: "<< camera_frame);
    }

    if(!n.getParam("x_min", x_min)) {
        ROS_ERROR("Must provide x-min limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("x_min: "<< x_min);
    }

    if(!n.getParam("x_max", x_max)) {
        ROS_ERROR("Must provide x-max limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("x_max: "<< x_max);
    }

    if(!n.getParam("y_min", y_min)) {
        ROS_ERROR("Must provide y-min limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("y_min: "<< y_min);
    }

    if(!n.getParam("y_max", y_max)) {
        ROS_ERROR("Must provide y-max limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("y_max: "<< y_max);
    }

    if(!n.getParam("z_min", z_min)) {
        ROS_ERROR("Must provide z-min limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("z_min: "<< z_min);
    }

    if(!n.getParam("z_max", z_max)) {
        ROS_ERROR("Must provide z-max limit!");
        ret = false;
    } else {
        ROS_INFO_STREAM("z_max: "<< z_max);
    }

    if(!n.getParam("stat_mean", stat_mean)) {
        ROS_ERROR("Must provide Statistical Outlier expected mean!");
        ret = false;
    } else {
        ROS_INFO_STREAM("stat_mean: "<< stat_mean);
    }

    if(!n.getParam("stat_std", stat_std)) {
        ROS_ERROR("Must provide Statistical Outlier expected standard deviation!");
        ret = false;
    } else {
        ROS_INFO_STREAM("stat_std: "<< stat_std);
    }

    if(!n.getParam("pcl_viz", pcl_viz)) {
        ROS_ERROR("Must provide bool to start bringup pcl visualization!");
        ret = false;
    } else {
        ROS_INFO_STREAM("pcl_viz: "<< pcl_viz);
    }


    return ret;
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  //-----------------------ROS TO PCL CONVERSIONS --------------------------//
  //-- Convert to the templated PointCloud --//
  pcl::PointCloud<pcl::PointXYZRGB> cloudxx;

  // Convert ros msg cloud to pcl cloud
  pcl::fromROSMsg(*msg,cloudxx);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(cloudxx));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);


  //Transform Point Cloud to Base RF
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  tf::StampedTransform base2kinect, base2left, base2right;
  string world_frame ("/world_frame"), left_frame("/left_arm_flange_link"), right_frame("/right_arm_flange_link");

  try{
        // Camera Frame
        listener.waitForTransform(world_frame,camera_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(world_frame,camera_frame, ros::Time(0), base2kinect);

        // Left Arm Frame
        listener.waitForTransform(world_frame,left_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(world_frame,left_frame, ros::Time(0), base2left);

        // Right Arm Frame
        listener.waitForTransform(world_frame,right_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(world_frame,right_frame, ros::Time(0), base2right);
      }
   catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

  //-----------------------PCL PROCESSING --------------------------//

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

  //-- Removing Outliers in Scene --//
  if (stat_mean>1){
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (filtered_cloud_ptr);
      sor.setMeanK (stat_mean);
      sor.setStddevMulThresh (stat_std);
      sor.filter (*filtered_cloud_ptr);
  }

  //TODO: Give option to do this or not
  //-- Publish the table point cloud --//
  pcl::PCLPointCloud2 output;
  pcl::toPCLPointCloud2(*filtered_cloud_ptr, output);
  pub.publish (output);


  //-> Use Base RF for Bounding Box
  tf::Transform b2k = base2kinect;
  tf::Transform kinect2base = base2kinect.inverse();
  Eigen::Affine3d k2b_eig, b2k_eig;
  tf::transformTFToEigen(kinect2base,k2b_eig);
  tf::transformTFToEigen(b2k,b2k_eig);
  pcl::transformPointCloud (*filtered_cloud_ptr, *filtered_cloud_ptr, b2k_eig);
  if (pcl_viz)
     updateVis(visor, filtered_cloud_ptr);

  //-- Get Bounding Box of Table Setting --//
  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*filtered_cloud_ptr, minPt, maxPt);
  if (pcl_viz){
        visor->addCube(minPt.x,maxPt.x,minPt.y,maxPt.y,0.0,0.03,1,1,1,"table");
        visor->addCoordinateSystem();
  }


  //-- Calculate surface normals --//
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZRGB>());
//  normalEstimator.setInputCloud(filtered_cloud_ptr);
//  normalEstimator.setSearchMethod(treeNormals);
//  normalEstimator.setRadiusSearch (norm_rad);

  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
//  Eigen::Vector4f xyz_centroid;
//  compute3DCentroid (*filtered_cloud_ptr, xyz_centroid);
//  normalEstimator.setViewPoint(xyz_centroid[0],xyz_centroid[1],xyz_centroid[2]+0.3);
//  normalEstimator.compute(*cloud_normals);

////  // For Debugging
//  updateVis(visor, filtered_cloud_ptr, cloud_normals);

  //-- Table Plane Detection --//
//  pcl::PointIndices table_inliers;
//  pcl::ModelCoefficients table_coeff;
//  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB,pcl::Normal> tableSegmentor;
//  tableSegmentor.setOptimizeCoefficients(true);
//  tableSegmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//  tableSegmentor.setMethodType(pcl::SAC_RANSAC);
//  tableSegmentor.setProbability(0.99);
//  tableSegmentor.setDistanceThreshold(0.01);
//  tableSegmentor.setInputCloud(filtered_cloud_ptr);
//  tableSegmentor.setInputNormals(cloud_normals);
//  tableSegmentor.segment(table_inliers,table_coeff);

  //-- Project Inliers to the Estimated Table Plan --//
//  pcl::PointCloud<pcl::PointXYZRGB> table_projected;
//  pcl::PointIndicesPtr table_inliers_ptr (new pcl::PointIndices(table_inliers));
//  pcl::ModelCoefficients::Ptr table_coeff_ptr (new pcl::ModelCoefficients(table_coeff));
//  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//  proj.setInputCloud(filtered_cloud_ptr);
//  proj.setIndices(table_inliers_ptr);
//  proj.setModelCoefficients(table_coeff_ptr);
//  proj.filter(table_projected);


  //-- Get Bounding Box of Table Setting --//
//  pcl::PointXYZRGB minPt, maxPt;
//  pcl::getMinMax3D (table_projected, minPt, maxPt);
//  std::cout << "Max x: " << maxPt.x << std::endl;
//  std::cout << "Max y: " << maxPt.y << std::endl;
//  std::cout << "Max z: " << maxPt.z << std::endl;
//  std::cout << "Min x: " << minPt.x << std::endl;
//  std::cout << "Min y: " << minPt.y << std::endl;
//  std::cout << "Min z: " << minPt.z << std::endl;

//  visor->addCube(minPt.x,maxPt.x,minPt.y,maxPt.y,minPt.z,maxPt.z,1,1,1,"table");

//  //-- Estimate the Convex Hull of the Projected Points --//
//    pcl::ConvexHull<pcl::PointXYZRGB> hull;
//    hull.setInputCloud(filtered_cloud_ptr);
//    hull.reconstruct(*table_hull_ptr);

//  // For Debugging
//    updateVis(visor, table_hull_ptr);

  // Creating the KdTree object for the search method of the extraction
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//  tree->setInputCloud (filtered_cloud_ptr);

//  std::vector<pcl::PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//  ec.setClusterTolerance (0.02); // 2cm
//  ec.setMinClusterSize (100);
//  ec.setMaxClusterSize (25000);
//  ec.setSearchMethod (tree);
//  ec.setInputCloud (filtered_cloud_ptr);
//  ec.extract (cluster_indices);

//  int j = 0;
//  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//  {
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
//    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//      cloud_cluster->points.push_back (filtered_cloud_ptr->points[*pit]); //*
//    cloud_cluster->width = cloud_cluster->points.size ();
//    cloud_cluster->height = 1;
//    cloud_cluster->is_dense = true;

//    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//    j++;
//  }

  //-----------------------PCL PROCESSING STEPS END HERE --------------------------//

  //-- Publish the table point cloud --//
//  pcl::PCLPointCloud2 output;
//  pcl::toPCLPointCloud2(*filtered_cloud_ptr, output);
//  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "filter_table_scene");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  //-- Parse Parameters from Server --//
  if(!parseParams(_nh)) {
        ROS_ERROR("Errors while parsing arguments.");
        return 1;
  }


  //-- Creat Visualizer Instance --//
  if (pcl_viz)
    visor = createVis();

  //-- Create a ROS subscriber for the input point cloud --//
  ros::Subscriber sub = nh.subscribe (input_cloud, 1, cloud_cb);

  //-- Create a ROS publisher for the output point cloud --//
  pub = nh.advertise<sensor_msgs::PointCloud2> (output_cloud, 1);

  // Spin
  if (pcl_viz){
      while(!visor->wasStopped()){
          sleep(1);
          ros::spin ();
      }
  }
  else{
  ros::spin ();}



}
