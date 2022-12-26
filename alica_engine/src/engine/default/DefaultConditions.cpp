#include "engine/default/DefaultConditions.h"

#include "engine/RunningPlan.h"
#include "engine/blackboard/Blackboard.h"

namespace alica
{
bool defaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return false;
}
} /* namespace alica */
