#include "SurroundLeaderTutorial.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

SurroundLeaderTutorial::SurroundLeaderTutorial(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SurroundLeaderTutorial::onInit() {}

std::unique_ptr<SurroundLeaderTutorial> SurroundLeaderTutorial::create(alica::PlanContext& context)
{
    return std::make_unique<SurroundLeaderTutorial>(context);
}

namespace
{

bool isSuccess(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Success;
    } else {
        return rp->getActiveState()->isSuccessState();
    }
}

} // namespace

bool TransitionConditions::CustomAnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isSuccess(child)) {
            return true;
        }
    }
    return false;
}

} // namespace turtlesim
