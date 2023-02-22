#include "Move.h"

namespace turtlesim
{
// Plan:  Move (1889749086610694100)
//
// Tasks:
//   - Follower (3759439551323513525) (Entrypoint: 3277312192440194145)//   - Leader (826983480584534597) (Entrypoint: 4346694000146342467)
//
// States:
//   - AlignCircle (2299237921449867536)
//   - Move2Center (4158797811607100614)
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
