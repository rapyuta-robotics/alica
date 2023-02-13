#include "OrderedSchedulingTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

OrderedSchedulingTestPlan::OrderedSchedulingTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void OrderedSchedulingTestPlan::onInit() {}

std::unique_ptr<OrderedSchedulingTestPlan> OrderedSchedulingTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<OrderedSchedulingTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> OrderedSchedulingTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<OrderedSchedulingTestPlanUtilityFunction> OrderedSchedulingTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<OrderedSchedulingTestPlanUtilityFunction>();
}

} // namespace alica::tests
