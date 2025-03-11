#include "WaitForTrigger.h"
#include "ros/console.h"

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
    _warning_timeout = ros::Duration{bb.get<double>("warning_timeout")};
    ROS_INFO_STREAM_NAMED(__func__, "Waiting for trigger on " << _topic);
    _triggerSub = ros::NodeHandle("~").subscribe(_topic, 1, &WaitForTrigger::onTrigger, this);
    _start_time = ros::Time::now();
}

void WaitForTrigger::run()
{
    if (isSuccess()) {
        return;
    }

    if (_triggered) {
        setSuccess();
    } else if (ros::Time::now() - _start_time > _warning_timeout) {
        ROS_WARN_STREAM_THROTTLE_NAMED(5, __func__, "Still waiting to receive trigger on " << _topic);
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
