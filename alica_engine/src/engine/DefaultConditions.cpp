#include "engine/DefaultConditions.h"

#include "engine/IAlicaWorldModel.h"
#include "engine/RunningPlan.h"
#include "engine/blackboard/Blackboard.h"

namespace alica
{
bool defaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return false;
}
} /* namespace alica */
