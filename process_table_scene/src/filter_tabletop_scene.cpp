#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string>
#include <stdio.h>

// Common
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// Filtering
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

// Visualization Shit
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// TF Stuff
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// Eigen Stuff
#include <Eigen/Core>
#include <Eigen/Geometry>

ros::Publisher pub1, pub2;
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_pcl;

///-- Declare Input/Output Cloud Names --///
string input_cloud(""), table_cloud(""), object_cloud(""), camera_frame(""), pcd_write_dir(""),
world_frame(""), arm_frame("");

///-- Declare PassFilter Parameters --///
double x_min(-1), x_max(1), y_min(-1), y_max(1), z_min(1), z_max(1.6);

///-- Declare StatOutliers Parameters --///
double stat_mean(10) , stat_std(1);

///-- Declare Plane Estimation Parameters --///
double norm_rad(0.03) ; //[m]

///-- Booleans to do stuff --///
bool pcl_viz(0), pub_rviz(0), table_fixed(1), write_pcd(0), valid_object(0), cutting_board(0);


///-- Transforms for Filters --///
Eigen::Affine3d k2b_eig, b2k_eig;
Eigen::Affine3d arm, arm_back;

///-- Parameter Reader --///
bool parseParams(const ros::NodeHandle& n) {
    bool ret = true;
    if(!n.getParam("input_cloud", input_cloud)) {
        ROS_ERROR("Must provide input cloud name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Input Cloud Name: "<< input_cloud);
    }

    if(!n.getParam("table_cloud", table_cloud)) {
        ROS_ERROR("Must provide table cloud name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Table Cloud Name: "<< table_cloud);
    }

    if(!n.getParam("object_cloud", object_cloud)) {
        ROS_ERROR("Must provide object cloud name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Object Cloud Name: "<< object_cloud);
    }

    if(!n.getParam("camera_frame_name", camera_frame)) {
        ROS_ERROR("Must provide camera frame name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Camera Frame Name: "<< camera_frame);
    }

    if(!n.getParam("world_frame", world_frame)) {
        ROS_ERROR("Must provide world frame name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("World Frame Name: "<< world_frame);
    }

    if(!n.getParam("arm_frame", arm_frame)) {
        ROS_ERROR("Must provide arm frame name!");
        ret = false;
    } else {
        ROS_INFO_STREAM("Arm Frame Name: "<< arm_frame);
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

    if(!n.getParam("table_fixed", table_fixed)) {
        ROS_ERROR("Must provide bool to table estimation!");
        ret = false;
    } else {
        ROS_INFO_STREAM("table_fixed: "<< table_fixed);
    }

    if(!n.getParam("cutting_board", cutting_board)) {
        ROS_ERROR("Must provide bool to segment and estimate cutting board tf!");
        ret = false;
    } else {
        ROS_INFO_STREAM("cutting_board: "<< cutting_board);
    }

    if(!n.getParam("pcl_viz", pcl_viz)) {
        ROS_ERROR("Must provide bool to start bringup pcl visualization!");
        ret = false;
    } else {
        ROS_INFO_STREAM("pcl_viz: "<< pcl_viz);
    }

    if(!n.getParam("pub_rviz", pub_rviz)) {
        ROS_ERROR("Must provide bool to publish to rviz!");
        ret = false;
    } else {
        ROS_INFO_STREAM("pub_rviz: "<< pub_rviz);
    }

    if(!n.getParam("write_pcd", write_pcd)) {
        ROS_ERROR("Must provide bool to write pcd!");
        ret = false;
    } else {
        ROS_INFO_STREAM("write_pcd: "<< write_pcd);
    }

    if(!n.getParam("pcd_write_dir", pcd_write_dir)) {
        ROS_ERROR("Must provide PCD Writing Directory !");
        ret = false;
    } else {
        ROS_INFO_STREAM("PCD Writing Directory: "<< pcd_write_dir);
    }

    return ret;
}


///-- PCL Visualization Functions --///
boost::shared_ptr<pcl::visualization::PCLVisualizer> visor;
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Zucchini Segmentation"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem (0.2);
  return viewer;
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    viewer->spinOnce();
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string& name){
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud(name);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
    viewer->spinOnce();
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb);
    viewer->spinOnce();
}


void updateVisNormals(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_,
               pcl::PointCloud<pcl::Normal>::ConstPtr normals_){
    viewer->removePointCloud("normals",0);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_, normals_, 10, 0.02, "normals");
    viewer->spinOnce();
}



///-- PCL Filtering Functions--///
void applyPassThroughFilter( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud_ptr ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud_ptr,
                           double& x_min, double& x_max,double& y_min,
                           double& y_max,double& z_min,double& z_max){

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (input_cloud_ptr);
    pass.setKeepOrganized(true);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter (*output_cloud_ptr);

    pass.setInputCloud(output_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter (*output_cloud_ptr);

    pass.setInputCloud(output_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter (*output_cloud_ptr);
}

void removeAxisAlignedBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud_ptr ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud_ptr,
                         pcl::PointXYZRGB&  minPt, pcl::PointXYZRGB& maxPt, bool negative){

    /// -- Create Octree to quickly search through bounding box -- ///
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
    std::vector<int>  table_indices;
    octree.setInputCloud (input_cloud_ptr);
    octree.addPointsFromInputCloud ();
    Eigen::Vector3f min(minPt.x,minPt.y,minPt.z);
    Eigen::Vector3f max(maxPt.x,maxPt.y,maxPt.z);
    octree.boxSearch(min, max, table_indices);

    /// -- Create the filtering object --- ///
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointIndicesPtr inliers (new pcl::PointIndices());
    for (uint i=0;i<table_indices.size();i++)
        inliers->indices.push_back(table_indices.at(i));

    ///-- Extract the inliers -- ///
    extract.setInputCloud (input_cloud_ptr);
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filter (*output_cloud_ptr);

}

void removeAxisAlignedBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud_ptr ,
                         pcl::PointXYZRGB&  minPt, pcl::PointXYZRGB& maxPt, bool negative){

    /// -- Create Octree to quickly search through bounding box -- ///
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
    std::vector<int>  table_indices;
    octree.setInputCloud (input_cloud_ptr);
    octree.addPointsFromInputCloud ();
    Eigen::Vector3f min(minPt.x,minPt.y,minPt.z);
    Eigen::Vector3f max(maxPt.x,maxPt.y,maxPt.z);
    octree.boxSearch(min, max, table_indices);

    /// -- Create the filtering object -- ///
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointIndicesPtr inliers (new pcl::PointIndices());
    for (uint i=0;i<table_indices.size();i++)
        inliers->indices.push_back(table_indices.at(i));

    /// -- Extract the inliers -- //
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filterDirectly(input_cloud_ptr);


}


const std::string currentDateTime() {
    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

void extract_table_object (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& table_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object_cloud_ptr,  pcl::PointXYZRGB& tab_minPt, pcl::PointXYZRGB& tab_maxPt)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    /// --- Remove background with pass-through filter --- ///
    /// \brief applyPassThroughFilter
    /// Removes Data from Point Cloud within the selected bounds
    ///
    applyPassThroughFilter(input_cloud_ptr , filtered_cloud_ptr, x_min, x_max, y_min, y_max, z_min, z_max);

    /// --- Removing Outliers in Scene --- ///
    /// \brief If desired, filtering out
    /// noisy points.
    ///
    if (stat_mean>1){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (filtered_cloud_ptr);
        sor.setMeanK (stat_mean);
        sor.setStddevMulThresh (stat_std);
        sor.filter (*filtered_cloud_ptr);
    }

    /// --- Create Table Pointcloud --- //
    /// \brief We begin by trasforming the cloud to the base RF
    /// then we either extract the table from fixed parameters
    /// i.e. (table_fixed=true) or with Bounding Box from minmax values
    /// -- only do this if table topic name is given
    pcl::transformPointCloud (*filtered_cloud_ptr, *filtered_cloud_ptr, b2k_eig);
    if (cutting_board){// Params for Cutting Board
        tab_maxPt.x = -0.402856; tab_maxPt.y = -0.355587; tab_maxPt.z = 0.035;
        tab_minPt.x = -0.612339; tab_minPt.y = -0.659793; tab_minPt.z = -0.01;

    }
    else{              // Params for Big Table
        tab_maxPt.x = -0.182856; tab_maxPt.y = -0.155587; tab_maxPt.z = 0.035;
        tab_minPt.x = -0.752339; tab_minPt.y = -0.879793; tab_minPt.z = -0.01;
    }

    if (!table_fixed){
        //-- Get Bounding Boxes of Table Setting --//
        pcl::getMinMax3D (*filtered_cloud_ptr, tab_minPt, tab_maxPt);}

    if (table_cloud != ""){
        removeAxisAlignedBB(filtered_cloud_ptr, table_cloud_ptr, tab_minPt, tab_maxPt, false);
        std::vector<int> indices_tab;
        pcl::removeNaNFromPointCloud(*table_cloud_ptr,*table_cloud_ptr, indices_tab);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(tab_maxPt.x, tab_maxPt.y - 0.07, (tab_minPt.z + tab_maxPt.z)/2) );
        tf::Quaternion q (0,0,0,1);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, "/task_frame"));

    }

    /// --- Create Zucchini Pointcloud and Extract Bounding Box with PCA --- ///
    /// \brief Extract the remaining points ~sans the table. Then we transform
    /// the cloud to the Right Arm EE Frame and extract the bounding box
    /// corresponding to the object's (zucchini) location.
    ///
    removeAxisAlignedBB(filtered_cloud_ptr,object_cloud_ptr, tab_minPt, tab_maxPt, true);
    pcl::PointXYZRGB obj_minPt, obj_maxPt;
    obj_minPt.x = -0.06; obj_minPt.y = -0.06; obj_minPt.z = 0.18;
    obj_maxPt.x = 0.06;  obj_maxPt.y = 0.06;  obj_maxPt.z = 0.4;
    pcl::transformPointCloud(*object_cloud_ptr,*object_cloud_ptr, arm);
    removeAxisAlignedBB(object_cloud_ptr, obj_minPt, obj_maxPt, false);


    ///-- Transform Object back to Robot Base RF --///
    pcl::transformPointCloud(*object_cloud_ptr,*object_cloud_ptr, arm_back);
    std::vector<int> indices_obj;
    pcl::removeNaNFromPointCloud(*object_cloud_ptr,*object_cloud_ptr, indices_obj);

}




void process_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object_cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr&  object_normals_ptr,
                    pcl::PointCloud <pcl::PointXYZRGB>::Ptr& colored_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& final_object_ptr,
                    pcl::PointXYZRGBNormal& bb_minPoint, pcl::PointXYZRGBNormal& bb_maxPoint, Eigen::Quaternionf& bb_Quat, Eigen::Vector3f& bb_Transform)
{

    /// -- Apply Region Growing and Euclidean Distance Filter to Find the
    /// Smooth Continuous Surface of the Zucchini -- //
    /// \input  object_cloud_ptr
    /// \output object_cloud_ptr
    ///

    if (object_cloud_ptr->points.size() > 100){

        ///-- Calculate surface normals of Object (Zucchini) --//
        /// \brief We compute this in order to help feature tracking
        /// and reconstruction functions.
        ///
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZRGB>());
        normalEstimator.setInputCloud(object_cloud_ptr);
        normalEstimator.setSearchMethod(treeNormals);
        normalEstimator.setRadiusSearch (norm_rad);
        Eigen::Vector4f xyz_centroid;
        compute3DCentroid (*object_cloud_ptr, xyz_centroid);
        normalEstimator.setViewPoint(xyz_centroid[0],xyz_centroid[1],xyz_centroid[2]+0.3);
        normalEstimator.compute(*object_normals_ptr);
        pcl::concatenateFields (*object_cloud_ptr, *object_normals_ptr, *final_object_ptr);

        /// -- Check for point-to-point Euclidean Distances -- ///
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree->setInputCloud (final_object_ptr);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
        ec.setInputCloud (final_object_ptr);
        ec.setClusterTolerance (0.015); // 1cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.extract (cluster_indices);

        /// -- Extract Largest Cluster (TODO: Make Function) -- //
        int max_cluster = 0;
        int max_cluster_size = 0;
        for (int i = 0; i<cluster_indices.size();i++){
            if (cluster_indices[i].indices.size() > max_cluster_size){
                max_cluster = i;
                max_cluster_size = cluster_indices[i].indices.size();
            }
        }

        /// -- Extract Euclidean Clustering Inliers -- ///
        pcl::PointIndicesPtr ec_inliers (new pcl::PointIndices());
        for (uint i=0;i<cluster_indices[max_cluster].indices.size ();i++)
            ec_inliers->indices.push_back(cluster_indices[max_cluster].indices.at(i));


        /// -- Check for Smooth Consisten Surface with Normal-based Region Growing -- ///
        pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
        std::vector <pcl::PointIndices> clusters;
        reg.setMinClusterSize (100);
        reg.setMaxClusterSize (object_cloud_ptr->points.size());
        reg.setSearchMethod (treeNormals);
        reg.setNumberOfNeighbours (10);
        reg.setInputCloud (object_cloud_ptr);
        reg.setIndices(ec_inliers);
        reg.setInputNormals (object_normals_ptr);
        reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (0.5);
        reg.extract (clusters);

        /// -- Extract Largest Cluster (TODO: Make Function) -- //
        max_cluster = 0;
        max_cluster_size = 0;
        for (int i = 0; i<clusters.size();i++){
            if (clusters[i].indices.size() > max_cluster_size){
                max_cluster = i;
                max_cluster_size = clusters[i].indices.size();
            }
        }
        colored_cloud = reg.getColoredCloud ();

        /// -- Extract Region Growing Inliers -- ///
        pcl::PointIndicesPtr rg_inliers (new pcl::PointIndices());
        for (uint i=0;i<clusters[max_cluster].indices.size ();i++)
            rg_inliers->indices.push_back(clusters[max_cluster].indices.at(i));

        /// -- Create the filtering object -- ///
        pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
        /// -- Extract the inliers -- //
        extract.setIndices (rg_inliers);
        extract.setNegative (false);
        extract.filterDirectly(final_object_ptr);
        std::vector<int> indices_filt;
        pcl::removeNaNFromPointCloud(*final_object_ptr,*final_object_ptr, indices_filt);


        /// --- Compute principal directions of the Zucchini --- ///
        /// \brief Compute centroid and PCA to get the enclosing
        /// bounding box of the points belonging to the object
        ///
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*final_object_ptr, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*final_object_ptr, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// Necessary for proper orientation

        ///-- Transform object to PCA Basis --///
        /// \brief Transform the original cloud to the origin    pcl::PointXYZRGBNormal  bb_minPoint, bb_maxPoint;
        /// where the principal components correspond to the axes.
        ///
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_object_projected_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::transformPointCloud(*final_object_ptr, *final_object_projected_ptr, projectionTransform);

        ///-- Extract Bounding Box Parameters --///
        /// \brief By computing the minimum and
        /// maximum points of the transformed cloud
        /// in the PCA basis.
        ///
        pcl::getMinMax3D(*final_object_projected_ptr, bb_minPoint, bb_maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(bb_maxPoint.getVector3fMap() + bb_minPoint.getVector3fMap());
        const Eigen::Quaternionf bb_Quat2(eigenVectorsPCA);
        bb_Quat = Eigen::Quaternionf(bb_Quat2);
        bb_Transform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        ///-- Extract Bounding Box Parameters --///
        /// \brief publish object transform
        ///
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pcaCentroid[0], pcaCentroid[1], pcaCentroid[2]));
        tf::Quaternion q;
        tf::quaternionEigenToTF(bb_Quat.cast<double>(),q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, "/object_frame"));
        valid_object = true;

    }
    else{
         colored_cloud = object_cloud_ptr;
         valid_object = false;
    }


}

///-- PointCloud Callback ---//
void cloud_cb (const PointCloud_pcl::ConstPtr& cloud_in){


    /// --- Data containers for Point Cloud Processing -- ///
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_in));    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    /// ---Extracting Reference Frames of Camera wrt. Robot Base and Arm EE Frame ---///
    tf::TransformListener listener;
    tf::StampedTransform base2kinect, base2arm;
    try{
        // Camera Frame
        listener.waitForTransform(world_frame,camera_frame, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(world_frame,camera_frame, ros::Time(0), base2kinect);

        // Arm Frame
        listener.waitForTransform(world_frame, arm_frame, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(world_frame, arm_frame, ros::Time(0), base2arm);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    /// --- Apply Conversions from TF to Eigen --///
    /// \brief Annoying AF
    ///
    // For Robot Base-Kinect
    tf::Transform b2k = base2kinect;
    tf::Transform kinect2base = base2kinect.inverse();
    tf::transformTFToEigen(base2kinect.inverse(),k2b_eig);
    tf::transformTFToEigen(base2kinect,b2k_eig);

    // For Robot Base-Arm EE
    Eigen::Quaterniond arm_quat_conv;
    Eigen::Vector3d arm_pos_conv;
    tf::quaternionTFToEigen(base2arm.getRotation(), arm_quat_conv);
    tf::vectorTFToEigen(base2arm.getOrigin(), arm_pos_conv);
    tf::transformTFToEigen(base2arm.inverse(),arm);
    tf::transformTFToEigen(base2arm,arm_back);

    /// -- Extract Table and Object with BB Method -- ///
    /// \brief extract_table_object
    /// Takes in the raw input point cloud, applies
    /// filters to segment table and object as two
    /// new point clouds
    ///
    pcl::PointXYZRGB tab_minPt,  tab_maxPt;
    extract_table_object(input_cloud_ptr, table_cloud_ptr, object_cloud_ptr, tab_minPt,  tab_maxPt);

    /// -- Process Object Point Cloud to extract Smooth Surface -- ///
    /// \brief process_object
    /// Applies Region Growing and Euclidean Clustering algorithm to
    /// raw object point cloud.
    ///
    ///
    pcl::PointCloud<pcl::Normal>::Ptr object_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_object_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointXYZRGBNormal  bb_minPoint, bb_maxPoint;
    Eigen::Quaternionf bb_Quat;
    Eigen::Vector3f bb_Transform;
    process_object(object_cloud_ptr, object_normals_ptr, colored_cloud, final_object_ptr, bb_minPoint, bb_maxPoint, bb_Quat, bb_Transform);

    ///-- PCL Visualization -- ///
    /// \brief If true visualize clouds
    /// bounding boxes and normals in
    /// PCL visualizer window.
    ///
    if (pcl_viz){
        string table ("table_top"), object ("zucch") ;

        /// -- Visualize Table Pointcloud -- ///
        if (table_cloud != "")
            updateVis(visor, table_cloud_ptr, table);
        visor->removeShape(table);
        visor->addCube(tab_minPt.x,tab_maxPt.x,tab_minPt.y,tab_maxPt.y,tab_minPt.z,tab_maxPt.z,1,1,1,table);


        /// -- Visualize Object Pointcloud -- ///
        visor->removeShape(object);
        updateVis(visor,colored_cloud, object);
        if (valid_object){
            visor->addCube(bb_Transform, bb_Quat , bb_maxPoint.x - bb_minPoint.x, bb_maxPoint.y - bb_minPoint.y, bb_maxPoint.z - bb_minPoint.z, object);
            updateVisNormals(visor, object_cloud_ptr, object_normals_ptr);
        }

    }

    /// -- Point Cloud Publishing -- ///
    /// \brief If true point clouds will
    /// be published and can be visualized
    /// in RViz.
    ///
    if (pub_rviz){

        pcl::PCLPointCloud2 output;

        ///-- Publish table point cloud --///
        if (table_cloud != ""){
            pcl::transformPointCloud(*table_cloud_ptr,*table_cloud_ptr,k2b_eig);
            pcl::toPCLPointCloud2(*table_cloud_ptr,output);
            pub1.publish (output);
        }

        ///-- Publish object point cloud --///
        pcl::transformPointCloudWithNormals(*final_object_ptr,*final_object_ptr,k2b_eig);        
        pcl::toPCLPointCloud2(*final_object_ptr,output);
        pub2.publish (output);
    }

    ///-- Write First Extract Point Cloud --///
    /// \brief This is for debugging/modeling purposes.
    ///
    if (write_pcd){
        time_t t = time(0);

        string filename = pcd_write_dir + "zucchini-test-" + currentDateTime() + ".pcd";
        std::cout << "Wrote pcd file to :" << filename << std::endl;
        pcl::io::savePCDFileASCII (filename, *final_object_ptr);
        write_pcd = false;
    }

}

int
main (int argc, char** argv)
{
  ///-- ROS Initializations --//
  ros::init (argc, argv, "filter_table_scene");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  ///-- Parse Parameters from Server --///
  if(!parseParams(_nh)) {
        ROS_ERROR("Errors while parsing arguments.");
        return 1;
  }

  ///-- Creat Visualizer Instance --///
  if (pcl_viz)
    visor = createVis();

  ///-- Create a ROS subscriber for the input point cloud --///
  ros::Subscriber sub = nh.subscribe<PointCloud_pcl> (input_cloud, 1, cloud_cb, ros::TransportHints().tcpNoDelay());

  ///-- Create a ROS publishers for the output point clouds --///
  pub1 = nh.advertise<sensor_msgs::PointCloud2> (table_cloud, 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> (object_cloud, 1);

  if (pcl_viz){ // If PCL Visualization is instantiated
      while(!visor->wasStopped()){
          ros::spin ();}
  }
  else{        // Otherwise
  ros::spin ();}

  ros::shutdown();
  return 0;

}
