#include "WaitForTrigger.h"

using std::placeholders::_1;

namespace ros_utils
{

WaitForTrigger::WaitForTrigger(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
        , _triggered{false}
{
}

void WaitForTrigger::initialiseParameters()
{
    _triggered = false;
    alica::LockedBlackboardRO bb{*getBlackboard()};
    _topic = bb.get<std::string>("topic");
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("~");
    _triggerSub = nh->create_subscription<std_msgs::msg::Empty>(_topic, 1, std::bind(&WaitForTrigger::onTrigger, this, _1));
}

void WaitForTrigger::run()
{
    if (isSuccess()) {
        return;
    }

    if (_triggered) {
        setSuccess();
    }
}

std::unique_ptr<WaitForTrigger> WaitForTrigger::create(alica::BehaviourContext& context)
{
    return std::make_unique<WaitForTrigger>(context);
}

void WaitForTrigger::onTrigger(const std_msgs::msg::Empty& triggerMsg)
{
    _triggered = true;
}

} // namespace ros_utils
