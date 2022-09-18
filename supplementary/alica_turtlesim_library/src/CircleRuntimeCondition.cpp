#pragma once

#include "CircleRuntimeCondition.h"

namespace alica
{
bool CircleRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    return true;
}

} // namespace alica
