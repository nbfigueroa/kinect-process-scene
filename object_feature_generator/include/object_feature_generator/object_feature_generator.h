#ifndef OBJECT_FEATURE_GENERATOR_H
#define OBJECT_FEATURE_GENERATOR_H

#include <string>
#include <math.h>

// ROS Stuff
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>

// PCL Stuff
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>

// Eigen Stuff
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud_pcl;

class ObjectFeatureGenerator
{

private:

    ros::NodeHandle nh;
    pcl::visualization::PCLVisualizer::Ptr visualizer;

    ros::Subscriber sub;
    ros::Publisher  pub;

    std::string cloud_topic, feat_topic;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
    bool runViz, running;
    int size;


public:

    ObjectFeatureGenerator (const std::string cloudTopic, const std::string featTopic, const bool runVisualizer) ;

    ~ObjectFeatureGenerator();

    void run();

    void cloudCallback (const PointCloud_pcl::ConstPtr& cloud_in);

    void sendFeatures(const Eigen::MatrixXd mean_rgb, const Eigen::MatrixXd std_rgb);

    void viewObject();
//    {
//        if (running){
//            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
//            visualizer->removeAllPointClouds();
//            visualizer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, cloud_topic);
//            visualizer->spinOnce();
//        }
//        else
//            visualizer->close();
//    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);
//    {
//        if(event.keyUp())
//        {
//            switch(event.getKeyCode())
//            {
//            case 27:
//            case 'q':
//                running = false;
//                break;
//            case ' ':
//                break;
//            }
//        }
//    }

};


#endif // OBJECT_FEATURE_GENERATOR_H
