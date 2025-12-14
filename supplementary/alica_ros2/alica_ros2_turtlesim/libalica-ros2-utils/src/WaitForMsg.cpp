#include "WaitForMsg.h"

using std::placeholders::_1;

namespace ros_utils
{

WaitForMsg::WaitForMsg(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void WaitForMsg::initialiseParameters()
{
    alica::LockedBlackboardRO bb{*getBlackboard()};
    _topic = bb.get<std::string>("topic");
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("alica_ros2_turtlesim");
    _sub = nh->create_subscription<std_msgs::msg::String>(_topic, 10, std::bind(&WaitForMsg::onMsg, this, _1));
}

std::unique_ptr<WaitForMsg> WaitForMsg::create(alica::BehaviourContext& context)
{
    return std::make_unique<WaitForMsg>(context);
}

void WaitForMsg::onMsg(const std_msgs::msg::String::SharedPtr msg)
{
    if (isSuccess()) {
        return;
    }
    alica::LockedBlackboardRW bb{*getBlackboard()};
    bb.set("msg", msg->data);
    setSuccess();
}

} // namespace ros_utils
