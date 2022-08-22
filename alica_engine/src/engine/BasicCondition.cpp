#include "engine/BasicCondition.h"
#include "engine/RunningPlan.h"

namespace alica
{

BasicCondition::BasicCondition() {}

BasicCondition::~BasicCondition() {}

bool BasicCondition::isStateTimedOut(const AlicaTime timeOut, std::shared_ptr<RunningPlan> rp) const
{
    if (rp->getStateStartTime() == AlicaTime::zero())
        return false;
    AlicaTime timeDiff = rp->getAlicaClock().now() - rp->getStateStartTime();
    return timeDiff > timeOut;
}

bool BasicCondition::isTimeOut(const AlicaTime timeOut, const AlicaTime startTime, std::shared_ptr<RunningPlan> rp) const
{
    if (startTime == AlicaTime::zero())
        return false;
    AlicaTime timeDiff = rp->getAlicaClock().now() - startTime;
    return timeDiff > timeOut;
}
} /* namespace alica */
