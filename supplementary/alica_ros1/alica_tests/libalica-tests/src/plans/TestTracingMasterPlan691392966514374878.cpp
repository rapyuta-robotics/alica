#include "TestTracingMasterPlan.h"

#include <alica_tests/TestWorldModel.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestTracingMasterPlan::TestTracingMasterPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestTracingMasterPlan::onInit() {}

std::unique_ptr<TestTracingMasterPlan> TestTracingMasterPlan::create(alica::PlanContext& context)
{
    return std::make_unique<TestTracingMasterPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> TestTracingMasterPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestTracingMasterPlanUtilityFunction> TestTracingMasterPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestTracingMasterPlanUtilityFunction>();
}

} // namespace alica::tests
