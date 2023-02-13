#include "MultiAgentTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MultiAgentTestPlan::MultiAgentTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MultiAgentTestPlan::onInit() {}

std::unique_ptr<MultiAgentTestPlan> MultiAgentTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<MultiAgentTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> MultiAgentTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MultiAgentTestPlanUtilityFunction> MultiAgentTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MultiAgentTestPlanUtilityFunction>();
}

} // namespace alica::tests
