#include "SuccessOnCondWrapperBPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SuccessOnCondWrapperBPlan::SuccessOnCondWrapperBPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SuccessOnCondWrapperBPlan::onInit() {}

std::unique_ptr<SuccessOnCondWrapperBPlan> SuccessOnCondWrapperBPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SuccessOnCondWrapperBPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SuccessOnCondWrapperBPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SuccessOnCondWrapperBPlanUtilityFunction> SuccessOnCondWrapperBPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SuccessOnCondWrapperBPlanUtilityFunction>();
}

} // namespace alica::tests
