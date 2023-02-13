#include "TestTracingSubPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestTracingSubPlan::TestTracingSubPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestTracingSubPlan::onInit() {}

std::unique_ptr<TestTracingSubPlan> TestTracingSubPlan::create(alica::PlanContext& context)
{
    return std::make_unique<TestTracingSubPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> TestTracingSubPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestTracingSubPlanUtilityFunction> TestTracingSubPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestTracingSubPlanUtilityFunction>();
}

} // namespace alica::tests
