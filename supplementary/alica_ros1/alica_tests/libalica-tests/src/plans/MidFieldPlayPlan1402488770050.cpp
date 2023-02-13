#include "MidFieldPlayPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MidFieldPlayPlan::MidFieldPlayPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MidFieldPlayPlan::onInit() {}

std::unique_ptr<MidFieldPlayPlan> MidFieldPlayPlan::create(alica::PlanContext& context)
{
    return std::make_unique<MidFieldPlayPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> MidFieldPlayPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MidFieldPlayPlanUtilityFunction> MidFieldPlayPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MidFieldPlayPlanUtilityFunction>();
}

bool RunTimeCondition1402489260911::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

} // namespace alica::tests
