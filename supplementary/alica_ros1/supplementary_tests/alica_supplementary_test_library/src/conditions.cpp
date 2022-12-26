#include "conditions.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{
bool conditionVariableHandlingStart(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return false;
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return false;
}
} /* namespace alica */
