#ifndef OBJECT_FEATURE_GENERATOR_H
#define OBJECT_FEATURE_GENERATOR_H


#include <ros/ros.h>
#include <string>
#include <math.h>

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

    std::string cloud_topic;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
    bool runViz, running;
    int size;


public:

    ObjectFeatureGenerator (const std::string cloudTopic, const bool runVisualizer) : cloud_topic(cloudTopic), runViz(runVisualizer)
    {
        if (runViz){
            ROS_INFO_STREAM("In Vis" );
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Object Viewer"));
            visualizer = viewer;
            visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_topic);
            visualizer->initCameraParameters();
            visualizer->setBackgroundColor(0, 0, 0);
            visualizer->setSize(200, 200);
            visualizer->setShowFPS(true);
            visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
            visualizer->registerKeyboardCallback(&ObjectFeatureGenerator::keyboardEvent, *this);
        }

    }

    ~ObjectFeatureGenerator()
    {

    }

    void run()
    {
        running = true;

        sub = nh.subscribe<PointCloud_pcl> (cloud_topic, 1, &ObjectFeatureGenerator::cloudCallback, this);
        std::string r_ct = nh.resolveName (cloud_topic);
        ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );

        ros::spin();
    }

    void cloudCallback (const PointCloud_pcl::ConstPtr& cloud_in)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>(*cloud_in));
        cloud = cloud_in_ptr;
        size = cloud->points.size();
        ROS_INFO_STREAM("Received cloud with " << size << " points." );

        if (runViz)
            viewObject();

        ///-- Extract Color into --///
        Eigen::MatrixXi rgb(size,3);
        Eigen::MatrixXf hsv(size,3);        

        pcl::PointXYZRGBNormal point;
        Eigen::Vector3i rgb_;
        Eigen::Vector3f hsv_;
        for (uint i=0;i<size;i++){
            point = cloud->points.at(i);
            rgb_ = point.getRGBVector3i();

            rgb_to_hsv(rgb_,hsv_);
            rgb(i,0) = rgb_[0];rgb(i,1) = rgb_[1];rgb(i,2) = rgb_[2];
            hsv(i,0) = hsv_[0];hsv(i,1) = hsv_[1];hsv(i,2) = hsv_[2];
        }

        Eigen::VectorXi r(size);
        r = rgb.col(0);
        std::cout << r << std::endl;
//        std::cout << rgb << std::endl;
//        std::cout << hsv << std::endl;
    }


    void viewObject()
    {
        if (running){
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
            visualizer->removeAllPointClouds();
            visualizer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, cloud_topic);
            visualizer->spinOnce();
        }
        else
            visualizer->close();
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
    {
        if(event.keyUp())
        {
            switch(event.getKeyCode())
            {
            case 27:
            case 'q':
                running = false;
                break;
            case ' ':
                break;
            }
        }
    }

    /* math adapted from: http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
     * reasonably optimized for speed, without going crazy */
    void rgb_to_hsv (Eigen::Vector3i rgb, Eigen::Vector3f& hsv)
    {

        int r =  rgb[0]; int g =  rgb[1]; int b =  rgb[2];
        float r_h, r_s, r_v;
        float rp, gp, bp, cmax, cmin, delta, l;
        int cmaxwhich, cminwhich;
        float HUE_ANGLE = 60;

        rp = ((float) r) / 255;
        gp = ((float) g) / 255;
        bp = ((float) b) / 255;

        //debug ("rgb=%d,%d,%d rgbprime=%f,%f,%f", r, g, b, rp, gp, bp);

        cmax = rp;
        cmaxwhich = 0; /* faster comparison afterwards */
        if (gp > cmax) { cmax = gp; cmaxwhich = 1; }
        if (bp > cmax) { cmax = bp; cmaxwhich = 2; }
        cmin = rp;
        cminwhich = 0;
        if (gp < cmin) { cmin = gp; cminwhich = 1; }
        if (bp < cmin) { cmin = bp; cminwhich = 2; }

        //debug ("cmin=%f,cmax=%f", cmin, cmax);
        delta = cmax - cmin;

        /* HUE */
        if (delta == 0) {
            r_h = 0;
        } else {
            switch (cmaxwhich) {
            case 0: /* cmax == rp */
                r_h = HUE_ANGLE * (fmod ((gp - bp) / delta, 6));
                break;

            case 1: /* cmax == gp */
                r_h = HUE_ANGLE * (((bp - rp) / delta) + 2);
                break;

            case 2: /* cmax == bp */
                r_h = HUE_ANGLE * (((rp - gp) / delta) + 4);
                break;
            }
            if (r_h < 0)
                r_h += 360;
        }

        /* LIGHTNESS/VALUE */
        //l = (cmax + cmin) / 2;
        r_v = cmax;

        /* SATURATION */
        /*if (delta == 0) {
        *r_s = 0;
      } else {
        *r_s = delta / (1 - fabs (1 - (2 * (l - 1))));
      }*/
        if (cmax == 0) {
            r_s = 0;
        } else {
            r_s = delta / cmax;
        }
        //debug ("rgb=%d,%d,%d ---> hsv=%f,%f,%f", r, g, b, *r_h, *r_s, *r_v);
        hsv[0] = r_h;
        hsv[1] = r_s;
        hsv[2] = r_v;
    }


//    void hsv_to_rgb (float h, float s, float v, int *r_r, int *r_g, int *r_b) {
//      if (h > 360)
//        h -= 360;
//      if (h < 0)
//        h += 360;
//      h = CLAMP (h, 0, 360);
//      s = CLAMP (s, 0, 1);
//      v = CLAMP (v, 0, 1);
//      float c = v * s;
//      float x = c * (1 - fabsf (fmod ((h / HUE_ANGLE), 2) - 1));
//      float m = v - c;
//      float rp, gp, bp;
//      int a = h / 60;

//      //debug ("h=%f, a=%d", h, a);

//      switch (a) {
//        case 0:
//          rp = c;
//          gp = x;
//          bp = 0;
//        break;

//        case 1:
//          rp = x;
//          gp = c;
//          bp = 0;
//        break;

//        case 2:
//          rp = 0;
//          gp = c;
//          bp = x;
//        break;

//        case 3:
//          rp = 0;
//          gp = x;
//          bp = c;
//        break;

//        case 4:
//          rp = x;
//          gp = 0;
//          bp = c;
//        break;

//        default: // case 5:
//          rp = c;
//          gp = 0;
//          bp = x;
//        break;
//      }

//      *r_r = (rp + m) * 255;
//      *r_g = (gp + m) * 255;
//      *r_b = (bp + m) * 255;

//      //debug ("hsv=%f,%f,%f, ---> rgb=%d,%d,%d", h, s, v, *r_r, *r_g, *r_b);
//    }


};




#endif // OBJECT_FEATURE_GENERATOR_H
