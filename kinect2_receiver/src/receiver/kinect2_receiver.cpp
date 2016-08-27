#include "kinect2_receiver.h"
#include "kinect2_definitions.h"


Kinect2_Receiver::Kinect2_Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useCompressed(useCompressed),
      updateCloud(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh)
{
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
    publishCloud = false;


}

Kinect2_Receiver::~Kinect2_Receiver()
{
}

void Kinect2_Receiver::run_viewer()
{
    Kinect2_Receiver::start();
    Kinect2_Receiver::stop();
}

void Kinect2_Receiver::run_publisher(const std::string &topicCloud_, const bool withViewer_)
{
    topicCloud = topicCloud_;
    withViewer = withViewer_;
    publishCloud = true;
    Kinect2_Receiver::start();
    Kinect2_Receiver::stop();
}

void Kinect2_Receiver::start()
{
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    ROS_INFO_STREAM("topicCameraInfoColor: " << topicCameraInfoColor);
    ROS_INFO_STREAM("topicCameraInfoDepth: " << topicCameraInfoDepth);


    cloudFrame = "/" K2_DEFAULT_NS  K2_TF_IR_OPT_FRAME;

    // Image Subscribers
    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    // Image Synchronizers
    syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
    syncExact->registerCallback(boost::bind(&Kinect2_Receiver::callback, this, _1, _2, _3, _4));

    // Cloud Publisher
    if(publishCloud)
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> (topicCloud, 1);

    spinner.start();
    std::chrono::milliseconds duration(1);
    while(!updateCloud)
    {
        if(!ros::ok())
        {
            return;
        }
        std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    // Visualize/Publish Point Cloud
    if (publishCloud && !withViewer)
        cloudPublisher();
    else
        cloudViewer();
}


void Kinect2_Receiver::stop()
{
    Kinect2_Receiver::spinner.stop();

    delete syncExact;
    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
}

void Kinect2_Receiver::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
  cv::Mat color, depth;

  readCameraInfo(cameraInfoColor, cameraMatrixColor);
  readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
  readImage(imageColor, color);
  readImage(imageDepth, depth);

  // IR image input
  if(color.type() == CV_16U)
  {
    cv::Mat tmp;
    color.convertTo(tmp, CV_8U, 0.02);
    cv::cvtColor(tmp, color, CV_GRAY2BGR);
  }

  lock.lock();
  this->color = color;
  this->depth = depth;
  updateCloud = true;
  lock.unlock();
}


void Kinect2_Receiver::generateCloud()
{
    cv::Mat color, depth;
    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();
    createCloud(depth, color, cloud);
}

void Kinect2_Receiver::sendCloud()
{
    cloud->header.frame_id = cloudFrame;
    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*cloud,output);
    pubCloud.publish (output);
}

void Kinect2_Receiver::cloudViewer()
{
  pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Kinect2 Viewer"));
  const std::string cloudName = "rendered";

  generateCloud();

  visualizer->addPointCloud(cloud, cloudName);
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
  visualizer->initCameraParameters();
  visualizer->setBackgroundColor(0, 0, 0);
  visualizer->setSize(color.cols, color.rows);
  visualizer->setShowFPS(true);
  visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
  visualizer->registerKeyboardCallback(&Kinect2_Receiver::keyboardEvent, *this);

  for(; running && ros::ok();)
  {
    if(updateCloud)
    {
      generateCloud();

      visualizer->updatePointCloud(cloud, cloudName);

      if(publishCloud)
        sendCloud();
    }

    visualizer->spinOnce(10);
  }
  visualizer->close();
}

void Kinect2_Receiver::cloudPublisher()
{
    generateCloud();
    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        generateCloud();
        sendCloud();
      }
    }

}


void Kinect2_Receiver::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
{
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
  pCvImage->image.copyTo(image);
}

void Kinect2_Receiver::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
{
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
}



void Kinect2_Receiver::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  #pragma omp parallel for
  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
    const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      if(*itD == 0)
      {
        // not valid
        itP->x = itP->y = itP->z = badPoint;
        itP->rgba = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
      itP->a = 255;
    }
  }
}



void Kinect2_Receiver::createLookup(size_t width, size_t height){
  const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
  const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
  const float cx = cameraMatrixColor.at<double>(0, 2);
  const float cy = cameraMatrixColor.at<double>(1, 2);
  float *it;

  lookupY = cv::Mat(1, height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}

void Kinect2_Receiver::keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *){
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
