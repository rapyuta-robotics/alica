#include "CountUp.h"

#include <engine/blackboard/Blackboard.h>
#include <engine/teammanager/TeamManager.h>

#include <iostream>

namespace minimal
{

CountUp::CountUp(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

std::unique_ptr<CountUp> CountUp::create(alica::BehaviourContext& context)
{
    return std::make_unique<CountUp>(context);
}

void CountUp::initialiseParameters()
{
    std::cout << "[CountUp] started" << std::endl;
}

void CountUp::run()
{
    alica::LockedBlackboardRW bb(*getGlobalBlackboard());
    int64_t next = bb.get<int64_t>(COUNTER_KEY) + 1;
    bb.set(COUNTER_KEY, next);
    // getTeamSize() counts this agent plus every teammate it has discovered.
    std::cout << "[CountUp] agent " << getOwnId() << " counter = " << next << " (team size " << getTeamManager().getTeamSize() << ")" << std::endl;

    if (next >= TARGET) {
        // Reporting success is what lets the plan progress past this behaviour.
        setSuccess();
    }
}

void CountUp::onTermination()
{
    std::cout << "[CountUp] stopped" << std::endl;
}

} // namespace minimal
