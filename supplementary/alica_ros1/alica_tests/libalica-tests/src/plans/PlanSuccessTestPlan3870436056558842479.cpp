#include "PlanSuccessTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanSuccessTestPlan::PlanSuccessTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanSuccessTestPlan::onInit() {}

std::unique_ptr<PlanSuccessTestPlan> PlanSuccessTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<PlanSuccessTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanSuccessTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanSuccessTestPlanUtilityFunction> PlanSuccessTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanSuccessTestPlanUtilityFunction>();
}

} // namespace alica::tests
