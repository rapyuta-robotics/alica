#include <alica/Move.h>

namespace alica
{
// Plan:  Move (1889749086610694100)
//
// Tasks:
//   - Follower (3759439551323513525) (Entrypoint: 3277312192440194145)//   - Leader (826983480584534597) (Entrypoint: 4346694000146342467)
//
// States:
//   - AlignCircle (2299237921449867536)
//   - Move2Center (4158797811607100614)
Move::Move(PlanContext& context)
        : BasicPlan(context)
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
bool RunTimeCondition1288817888979746811::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}
/**
 * Task: Follower  -> EntryPoint-ID: 3277312192440194145
 * Task: Leader  -> EntryPoint-ID: 4346694000146342467
 */
std::shared_ptr<UtilityFunction> UtilityFunction1889749086610694100::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<UtilityFunction1889749086610694100> UtilityFunction1889749086610694100::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<UtilityFunction1889749086610694100>();
}

} // namespace alica
