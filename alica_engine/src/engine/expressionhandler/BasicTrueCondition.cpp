#include "engine/expressionhandler/BasicTrueCondition.h"

namespace alica
{

BasicTrueCondition::BasicTrueCondition() {}

BasicTrueCondition::~BasicTrueCondition() {}

bool BasicTrueCondition::evaluate(std::shared_ptr<RunningPlan> rp)
{
    return true;
}

} /* namespace alica */
