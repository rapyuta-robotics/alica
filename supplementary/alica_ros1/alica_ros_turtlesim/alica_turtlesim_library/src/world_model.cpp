#include "world_model.hpp"
#include <geometry_msgs/Twist.h>

namespace turtlesim
{

ALICATurtleWorldModel* turtlesim::ALICATurtleWorldModel::wmInstance_ = nullptr;

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
