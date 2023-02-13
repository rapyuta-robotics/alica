#include "BehSuccessTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

BehSuccessTestPlan::BehSuccessTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void BehSuccessTestPlan::onInit() {}

std::unique_ptr<BehSuccessTestPlan> BehSuccessTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<BehSuccessTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> BehSuccessTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<BehSuccessTestPlanUtilityFunction> BehSuccessTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<BehSuccessTestPlanUtilityFunction>();
}

} // namespace alica::tests
