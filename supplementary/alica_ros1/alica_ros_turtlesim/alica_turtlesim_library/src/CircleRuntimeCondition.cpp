#include "CircleRuntimeCondition.h"

namespace alica
{

CircleRuntimeCondition::CircleRuntimeCondition()
{
    std::cerr << "Debug:"
              << "CircleRuntimeCondition created" << std::endl;
}

bool CircleRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    std::cerr << "Debug:"
              << "CircleRuntimeCondition::evaluate" << std::endl;
    return true;
}

} // namespace alica
