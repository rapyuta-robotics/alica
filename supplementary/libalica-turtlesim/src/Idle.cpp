#include "Idle.h"

#include <engine/RunningPlan.h>
#include <engine/logging/Logging.h>

namespace turtlesim
{

Idle::Idle(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

Idle::~Idle() {}

void Idle::run()
{
    if (isSuccess()) {
        return;
    }

    if (getPlanContext()->getAlicaClock().now() > _endIdleTime) {
        setSuccess();
    }
}

void Idle::initialiseParameters()
{
    double idle_seconds = alica::LockedBlackboardRO(*(getBlackboard())).get<double>("time");
    if (idle_seconds > 100000) {
        alica::Logging::logWarn("Idle") << "Idle time too large, capping at 100,000s";
        idle_seconds = 100000;
    } else if (idle_seconds < 0) {
        alica::Logging::logWarn("Idle") << "Idle time must be positive, using 0s";
        idle_seconds = 0;
    }
    int time_in_ms = static_cast<int>(idle_seconds * 1000);
    _endIdleTime = getPlanContext()->getAlicaClock().now() + alica::AlicaTime::milliseconds(time_in_ms);
}

std::unique_ptr<Idle> Idle::create(alica::BehaviourContext& context)
{
    return std::make_unique<Idle>(context);
}

} /* namespace turtlesim */
