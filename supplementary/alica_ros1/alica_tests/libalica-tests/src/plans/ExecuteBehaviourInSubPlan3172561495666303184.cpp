#include "ExecuteBehaviourInSubPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

ExecuteBehaviourInSubPlan::ExecuteBehaviourInSubPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void ExecuteBehaviourInSubPlan::onInit() {}

std::unique_ptr<ExecuteBehaviourInSubPlan> ExecuteBehaviourInSubPlan::create(alica::PlanContext& context)
{
    return std::make_unique<ExecuteBehaviourInSubPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> ExecuteBehaviourInSubPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<ExecuteBehaviourInSubPlanUtilityFunction> ExecuteBehaviourInSubPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<ExecuteBehaviourInSubPlanUtilityFunction>();
}

} // namespace alica::tests
