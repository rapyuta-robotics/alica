#include "Master.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

Master::Master(alica::PlanContext& context)
        : BasicPlan(context)
{
}

std::unique_ptr<Master> Master::create(alica::PlanContext& context)
{
    return std::make_unique<Master>(context);
}

std::shared_ptr<alica::UtilityFunction> MasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterUtilityFunction> MasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterUtilityFunction>();
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
