#include <alica_ros2_turtlesim/world_model.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace turtlesim
{
using std::placeholders::_1;
ALICATurtleWorldModel* ALICATurtleWorldModel::instance = nullptr;

ALICATurtleWorldModel* ALICATurtleWorldModel::get()
{
    return instance;
}

void ALICATurtleWorldModel::init(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh)
{
    if (!instance) {
        instance = new ALICATurtleWorldModel(nh, priv_nh);
    }
}

void ALICATurtleWorldModel::del()
{
    delete instance;
}

ALICATurtleWorldModel::ALICATurtleWorldModel(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh)
        : turtle(priv_nh)
{
    // initialize publisher, subscriber and service client.
    _initTriggerSub = nh->create_subscription<std_msgs::msg::Empty>("/init", 1, std::bind(&ALICATurtleWorldModel::initTriggerSubCallback, this, _1));

    // initialize attribute.
    _initTrigger = false;
}

ALICATurtleWorldModel::~ALICATurtleWorldModel() {}
void ALICATurtleWorldModel::initTriggerSubCallback(const std_msgs::msg::Empty& msg)
{
    _initTrigger = true;
}

} // namespace turtlesim
