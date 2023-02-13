#include "MultiPlanInstanceSuccessTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MultiPlanInstanceSuccessTestPlan::MultiPlanInstanceSuccessTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MultiPlanInstanceSuccessTestPlan::onInit() {}

std::unique_ptr<MultiPlanInstanceSuccessTestPlan> MultiPlanInstanceSuccessTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<MultiPlanInstanceSuccessTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> MultiPlanInstanceSuccessTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MultiPlanInstanceSuccessTestPlanUtilityFunction> MultiPlanInstanceSuccessTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MultiPlanInstanceSuccessTestPlanUtilityFunction>();
}

} // namespace alica::tests
