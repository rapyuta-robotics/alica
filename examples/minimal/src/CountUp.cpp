#include "CountUp.h"

#include "Profiling.h"

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
    ZoneScoped;
    // Runs on the timer thread that will also call run(), so naming it here
    // labels the whole behaviour track rather than an anonymous thread id.
    tracy::SetThreadName("CountUp behaviour");
    TracyMessageL("CountUp started");
    std::cout << "[CountUp] started" << std::endl;
}

void CountUp::run()
{
    ZoneScoped;

    int64_t next;
    {
        // Contention on the global blackboard is the one thing here that can
        // actually block, so it gets a zone of its own.
        ZoneScopedN("blackboard update");
        alica::LockedBlackboardRW bb(*getGlobalBlackboard());
        next = bb.get<int64_t>(COUNTER_KEY) + 1;
        bb.set(COUNTER_KEY, next);
    }
    // Plotted rather than logged, so the counter shows up as a curve along the
    // same timeline as the zones.
    TracyPlot("CountUp counter", next);
    // getTeamSize() counts this agent plus every teammate it has discovered.
    std::cout << "[CountUp] agent " << getOwnId() << " counter = " << next << " (team size " << getTeamManager().getTeamSize() << ")" << std::endl;

    if (next >= TARGET) {
        // Reporting success is what lets the plan progress past this behaviour.
        TracyMessageL("CountUp reached target");
        setSuccess();
    }
}

void CountUp::onTermination()
{
    ZoneScoped;
    TracyMessageL("CountUp stopped");
    std::cout << "[CountUp] stopped" << std::endl;
}

} // namespace minimal
