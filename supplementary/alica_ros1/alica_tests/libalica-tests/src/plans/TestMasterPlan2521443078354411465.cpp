#include "TestMasterPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestMasterPlan::TestMasterPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestMasterPlan::onInit() {}

std::unique_ptr<TestMasterPlan> TestMasterPlan::create(alica::PlanContext& context)
{
    return std::make_unique<TestMasterPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> TestMasterPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestMasterPlanUtilityFunction> TestMasterPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestMasterPlanUtilityFunction>();
}

} // namespace alica::tests
