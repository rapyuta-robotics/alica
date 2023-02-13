#include "SuccessOnInitPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SuccessOnInitPlan::SuccessOnInitPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SuccessOnInitPlan::onInit() {}

std::unique_ptr<SuccessOnInitPlan> SuccessOnInitPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SuccessOnInitPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SuccessOnInitPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SuccessOnInitPlanUtilityFunction> SuccessOnInitPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SuccessOnInitPlanUtilityFunction>();
}

} // namespace alica::tests
