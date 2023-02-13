#include "AttackPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

AttackPlan::AttackPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void AttackPlan::onInit() {}

std::unique_ptr<AttackPlan> AttackPlan::create(alica::PlanContext& context)
{
    return std::make_unique<AttackPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> AttackPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<AttackPlanUtilityFunction> AttackPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<AttackPlanUtilityFunction>();
}

} // namespace alica::tests
