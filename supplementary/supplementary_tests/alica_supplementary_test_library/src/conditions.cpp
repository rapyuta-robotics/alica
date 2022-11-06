#include "conditions.h"

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{
bool conditionVariableHandlingStart(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return false;
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return false;
}
} /* namespace alica */
