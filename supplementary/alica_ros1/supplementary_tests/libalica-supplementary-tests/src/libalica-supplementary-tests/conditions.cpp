#include "libalica-supplementary-tests/conditions.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

bool vhStartCondition = false;

namespace alica
{
bool conditionVariableHandlingStart295816226925111421(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return vhStartCondition;
}
bool conditionVariableHandlingStart(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return vhStartCondition;
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return false;
}
} /* namespace alica */
