#include "Move.h"

namespace turtlesim
{

Move::Move(alica::PlanContext& context)
        : alica::BasicPlan(context)
{
}
Move::~Move() {}
std::unique_ptr<Move> Move::create(alica::PlanContext& context)
{
    return std::make_unique<Move>(context);
}
// Check of RuntimeCondition - (Name): CircleRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool CircleRuntimeCondition::evaluate(std::shared_ptr<alica::RunningPlan> rp, const alica::Blackboard* gb)
{
    return true;
}

std::shared_ptr<CircleRuntimeCondition> CircleRuntimeCondition::create(alica::ConditionContext&)
{
    return std::make_shared<CircleRuntimeCondition>();
}

std::shared_ptr<alica::UtilityFunction> MoveUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MoveUtilityFunction> MoveUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MoveUtilityFunction>();
}

} // namespace turtlesim
