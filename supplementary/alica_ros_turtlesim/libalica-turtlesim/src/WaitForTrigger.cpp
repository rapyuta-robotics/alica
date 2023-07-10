#include "WaitForTrigger.h"

#include "turtle_interfaces.hpp"

namespace ros_utils
{

WaitForTrigger::WaitForTrigger(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void WaitForTrigger::initialiseParameters()
{
    alica::LockedBlackboardRO bb{*getBlackboard()};
    {
        // reset triggered
        alica::LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("triggered", false);
    }

    auto turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    turtle->subOnTrigger(bb.get<std::string>("topic"), getGlobalBlackboard());
}

void WaitForTrigger::run()
{
    if (isSuccess()) {
        return;
    }

    alica::LockedBlackboardRO gb(*getGlobalBlackboard());
    if (gb.get<bool>("triggered")) {
        setSuccess();
    }
}

std::unique_ptr<WaitForTrigger> WaitForTrigger::create(alica::BehaviourContext& context)
{
    return std::make_unique<WaitForTrigger>(context);
}

} // namespace ros_utils
