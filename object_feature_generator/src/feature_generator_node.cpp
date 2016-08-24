#include "object_feature_generator.h"

int
main (int argc, char **argv)
{
    ros::init(argc, argv, "object_feature_generator", ros::init_options::AnonymousName);

    if(!ros::ok())
    {
        return 0;
    }

    std::string topicCloud = "/scene/zucchini";
    ROS_INFO_STREAM("Cloud topic: "  << topicCloud );
    bool runVisualizer = true;

    ObjectFeatureGenerator feature_generator(topicCloud, runVisualizer);
    ROS_INFO_STREAM("starting feature generator...");
    feature_generator.run();

    ros::shutdown();
    return 0;
}
