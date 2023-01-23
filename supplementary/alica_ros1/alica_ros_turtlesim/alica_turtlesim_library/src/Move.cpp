#include "Move.h"
#include <engine/DefaultUtilityFunction.h>

namespace alica
{

Move::Move(PlanContext& context)
        : BasicPlan(context)
{
}
Move::~Move() {}

std::shared_ptr<UtilityFunction> MoveUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

} // namespace alica
