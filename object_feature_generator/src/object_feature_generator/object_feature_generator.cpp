#include "object_feature_generator.h"

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud_pcl;

ObjectFeatureGenerator::ObjectFeatureGenerator (const std::string cloudTopic, const std::string featTopic, const bool runVisualizer) :
    cloud_topic(cloudTopic), runViz(runVisualizer), feat_topic(featTopic)
{
    if (runViz){
        ROS_INFO_STREAM("In Vis" );
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Object Viewer"));
        visualizer = viewer;
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_topic);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(1, 1, 1);
        visualizer->setSize(200, 200);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&ObjectFeatureGenerator::keyboardEvent, *this);
    }

}

ObjectFeatureGenerator::~ObjectFeatureGenerator()
{

}


void ObjectFeatureGenerator::run(){
    running = true;

    sub = nh.subscribe<PointCloud_pcl> (cloud_topic, 1, &ObjectFeatureGenerator::cloudCallback, this);
    std::string r_ct = nh.resolveName (cloud_topic);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );

    pub = nh.advertise<geometry_msgs::WrenchStamped>(feat_topic, 1);

    ros::spin();
}


void ObjectFeatureGenerator::cloudCallback(const PointCloud_pcl::ConstPtr& cloud_in){
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>(*cloud_in));
    cloud = cloud_in_ptr;
    size = cloud->points.size();
    ROS_INFO_STREAM("Received cloud with " << size << " points." );

    if (runViz)
        viewObject();

    if (size > 0){

        ///-- Extracting Color Features from Segmented Object--///
        Eigen::MatrixXd rgb(size,3);
        pcl::PointXYZRGBNormal point;
        Eigen::Vector3i rgb_;
        for (int i=0;i<size;i++){
            point = cloud->points.at(i);
            rgb_ = point.getRGBVector3i();

            rgb(i,0) = (double)rgb_[0]; rgb(i,1) = (double)rgb_[1]; rgb(i,2) = (double)rgb_[2];

        }

        // Stats for RGB Channel
        Eigen::MatrixXd mean_rgb(1,3);
        mean_rgb = rgb.colwise().mean();

        //-- Compute covariance here
        Eigen::MatrixXd  centered   = rgb.rowwise() - rgb.colwise().mean();
        Eigen::MatrixXd covariance = (centered.adjoint() * centered) / double(size - 1);
        Eigen::MatrixXd std_rgb(1,3);
        std_rgb << sqrt(covariance.coeff(0,0)) ,  sqrt(covariance.coeff(1,1)) , sqrt(covariance.coeff(2,2));


        ///-- Publish Feature Message--///
        sendFeatures(mean_rgb, std_rgb);
    }


}

void ObjectFeatureGenerator::sendFeatures(const Eigen::MatrixXd mean_rgb, const Eigen::MatrixXd std_rgb){
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = mean_rgb.coeff(0);
    msg.wrench.force.y = mean_rgb.coeff(1);
    msg.wrench.force.z = mean_rgb.coeff(2);

    msg.wrench.torque.x = std_rgb.coeff(0);
    msg.wrench.torque.y = std_rgb.coeff(1);
    msg.wrench.torque.z = std_rgb.coeff(2);
    pub.publish(msg);

}


void ObjectFeatureGenerator::viewObject(){
    if (running){
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
        visualizer->removeAllPointClouds();
        visualizer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, cloud_topic);
        visualizer->spinOnce();
    }
    else
        visualizer->close();
}

void ObjectFeatureGenerator::keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *){
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
//    Eigen::Vector3f hsv_;
//        Eigen::MatrixXf hsv(size,3);
//            rgb_to_hsv(rgb_,hsv_);
//            hsv(i,0) = hsv_[0];hsv(i,1) = hsv_[1]*100;hsv(i,2) = hsv_[2]*100;
//        // Stats for HSV Channel
//        Eigen::MatrixXf mean_hsv(1,3);
//        mean_hsv = hsv.colwise().mean();
//    void rgb_to_hsv (Eigen::Vector3i rgb, Eigen::Vector3f& hsv)
//    {

//        int r =  rgb[0]; int g =  rgb[1]; int b =  rgb[2];
//        float r_h, r_s, r_v;
//        float rp, gp, bp, cmax, cmin, delta, l;
//        int cmaxwhich, cminwhich;
//        float HUE_ANGLE = 60;

//        rp = ((float) r) / 255;
//        gp = ((float) g) / 255;
//        bp = ((float) b) / 255;


//        cmax = rp;
//        cmaxwhich = 0; /* faster comparison afterwards */
//        if (gp > cmax) { cmax = gp; cmaxwhich = 1; }
//        if (bp > cmax) { cmax = bp; cmaxwhich = 2; }
//        cmin = rp;
//        cminwhich = 0;
//        if (gp < cmin) { cmin = gp; cminwhich = 1; }
//        if (bp < cmin) { cmin = bp; cminwhich = 2; }

//        delta = cmax - cmin;

//        /* HUE */
//        if (delta == 0) {
//            r_h = 0;
//        } else {
//            switch (cmaxwhich) {
//            case 0: /* cmax == rp */
//                r_h = HUE_ANGLE * (fmod ((gp - bp) / delta, 6));
//                break;

//            case 1: /* cmax == gp */
//                r_h = HUE_ANGLE * (((bp - rp) / delta) + 2);
//                break;

//            case 2: /* cmax == bp */
//                r_h = HUE_ANGLE * (((rp - gp) / delta) + 4);
//                break;
//            }
//            if (r_h < 0)
//                r_h += 360;
//        }

//        /* VALUE */
//        r_v = cmax;

//        /* SATURATION */
//        if (cmax == 0) {
//            r_s = 0;
//        } else {
//            r_s = delta / cmax;
//        }

//        hsv[0] = r_h;
//        hsv[1] = r_s;
//        hsv[2] = r_v;
//    }
