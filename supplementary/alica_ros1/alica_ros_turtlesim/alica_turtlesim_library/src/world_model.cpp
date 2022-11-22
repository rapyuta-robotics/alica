#include <alica_ros_turtlesim/world_model.hpp>
#include <geometry_msgs/Twist.h>

namespace turtlesim
{

//std::shared_ptr<turtlesim::ALICATurtleWorldModel> testwm=nu;;;

ALICATurtleWorldModel::ALICATurtleWorldModel(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
        : turtle(priv_nh)
{
    // initialize publisher, subscriber and service client.
    _initTriggerSub = nh.subscribe("init", 1, &ALICATurtleWorldModel::initTriggerSubCallback, this);

    // initialize attribute.
    _initTrigger = false;
}

ALICATurtleWorldModel::~ALICATurtleWorldModel() {}
void ALICATurtleWorldModel::initTriggerSubCallback(const std_msgs::EmptyConstPtr& msg)
{
    _initTrigger = true;
}

} // namespace turtlesim
