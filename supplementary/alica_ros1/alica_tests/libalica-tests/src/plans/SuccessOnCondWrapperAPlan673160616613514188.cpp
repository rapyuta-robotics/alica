#include "SuccessOnCondWrapperAPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SuccessOnCondWrapperAPlan::SuccessOnCondWrapperAPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SuccessOnCondWrapperAPlan::onInit() {}

std::unique_ptr<SuccessOnCondWrapperAPlan> SuccessOnCondWrapperAPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SuccessOnCondWrapperAPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SuccessOnCondWrapperAPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SuccessOnCondWrapperAPlanUtilityFunction> SuccessOnCondWrapperAPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SuccessOnCondWrapperAPlanUtilityFunction>();
}

} // namespace alica::tests
