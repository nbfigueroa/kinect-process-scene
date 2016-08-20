#include "kinect2_receiver.h"
#include "kinect2_definitions.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  // Default params
  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  std::string topicCloud = K2_TOPIC_SD;
  bool useCompressed = false;
  bool withViewer    = false;

  // Subscribing to
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  topicCloud = "/" + ns + topicCloud + "/points_rect";

  ROS_INFO_STREAM("topic color: "  << topicColor );
  ROS_INFO_STREAM("topic depth: "  << topicDepth );
  ROS_INFO_STREAM("topic cloud: "  << topicCloud );

  // Instantiate Receiver Class
  Kinect2_Receiver receiver(topicColor, topicDepth, useCompressed);
  ROS_INFO_STREAM("starting publisher...");
  receiver.run_publisher(topicCloud, withViewer);

  ros::shutdown();
  return 0;
}
