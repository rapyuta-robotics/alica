#include "WaitForMsg.h"

#include "turtle_interfaces.hpp"

namespace ros_utils
{

WaitForMsg::WaitForMsg(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void WaitForMsg::initialiseParameters()
{
    alica::LockedBlackboardRO bb{*getBlackboard()};
    auto turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    turtle->subOnMsg(bb.get<std::string>("topic"), getGlobalBlackboard());
}

std::unique_ptr<WaitForMsg> WaitForMsg::create(alica::BehaviourContext& context)
{
    return std::make_unique<WaitForMsg>(context);
}

void WaitForMsg::run()
{
    if (isSuccess()) {
        return;
    }
    alica::LockedBlackboardRO gb(*getGlobalBlackboard());
    if (gb.hasValue("msg")) {
        setSuccess();
    }
}

} // namespace ros_utils
