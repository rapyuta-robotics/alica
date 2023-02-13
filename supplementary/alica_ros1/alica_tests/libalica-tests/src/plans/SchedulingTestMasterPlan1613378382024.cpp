#include "SchedulingTestMasterPlan.h"

#include <alica_tests/test_sched_world_model.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestMasterPlan::SchedulingTestMasterPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestMasterPlan::onInit() {}

std::unique_ptr<SchedulingTestMasterPlan> SchedulingTestMasterPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestMasterPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestMasterPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestMasterPlanUtilityFunction> SchedulingTestMasterPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestMasterPlanUtilityFunction>();
}

} // namespace alica::tests
