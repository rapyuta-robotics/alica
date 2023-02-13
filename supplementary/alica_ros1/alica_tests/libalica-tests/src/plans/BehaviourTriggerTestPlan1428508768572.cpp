#include "BehaviourTriggerTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

BehaviourTriggerTestPlan::BehaviourTriggerTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void BehaviourTriggerTestPlan::onInit() {}

std::unique_ptr<BehaviourTriggerTestPlan> BehaviourTriggerTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<BehaviourTriggerTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> BehaviourTriggerTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<BehaviourTriggerTestPlanUtilityFunction> BehaviourTriggerTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<BehaviourTriggerTestPlanUtilityFunction>();
}

} // namespace alica::tests
