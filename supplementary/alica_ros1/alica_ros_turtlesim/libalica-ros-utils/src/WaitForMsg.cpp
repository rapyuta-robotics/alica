#include "WaitForMsg.h"

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
    ROS_INFO_STREAM_NAMED(__func__, "Waiting for msg on " << _topic);
    _sub = ros::NodeHandle("~").subscribe(_topic, 1, &WaitForMsg::onMsg, this);
}

std::unique_ptr<WaitForMsg> WaitForMsg::create(alica::BehaviourContext& context)
{
    return std::make_unique<WaitForMsg>(context);
}

void WaitForMsg::onMsg(const std_msgs::String& msg)
{
    if (isSuccess()) {
        return;
    }
    ROS_INFO_STREAM_NAMED(__func__, "Msg received on " << _topic);
    alica::LockedBlackboardRW bb{*getBlackboard()};
    bb.set("msg", msg.data);
    setSuccess();
}

} // namespace ros_utils
