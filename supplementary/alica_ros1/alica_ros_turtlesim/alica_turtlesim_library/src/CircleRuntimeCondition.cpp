#include "CircleRuntimeCondition.h"

namespace alica
{

CircleRuntimeCondition::CircleRuntimeCondition() {}

bool CircleRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    return true;
}

} // namespace alica
