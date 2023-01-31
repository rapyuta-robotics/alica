#include "WaitForTrigger.h"

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
    ROS_INFO_STREAM_NAMED(__func__, "Waiting for trigger on " << _topic);
    _triggerSub = ros::NodeHandle("~").subscribe(_topic, 1, &WaitForTrigger::onTrigger, this);
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

void WaitForTrigger::onTrigger(const std_msgs::Empty& triggerMsg)
{
    ROS_INFO_STREAM_NAMED(__func__, "Trigger received on " << _topic);
    _triggered = true;
}

} // namespace ros_utils
