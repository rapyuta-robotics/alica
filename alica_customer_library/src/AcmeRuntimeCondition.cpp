#pragma once

#include "AcmeRuntimeCondition.h"

namespace alica
{
bool AcmeRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    return true;
}

} // namespace alica
