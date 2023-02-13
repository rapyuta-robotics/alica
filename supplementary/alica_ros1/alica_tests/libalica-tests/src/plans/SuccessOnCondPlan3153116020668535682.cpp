#include "SuccessOnCondPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SuccessOnCondPlan::SuccessOnCondPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SuccessOnCondPlan::onInit() {}

std::unique_ptr<SuccessOnCondPlan> SuccessOnCondPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SuccessOnCondPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SuccessOnCondPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SuccessOnCondPlanUtilityFunction> SuccessOnCondPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SuccessOnCondPlanUtilityFunction>();
}

} // namespace alica::tests
