#include "SchedulingTestSequencePlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestSequencePlan::SchedulingTestSequencePlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestSequencePlan::onInit() {}

std::unique_ptr<SchedulingTestSequencePlan> SchedulingTestSequencePlan::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestSequencePlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestSequencePlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestSequencePlanUtilityFunction> SchedulingTestSequencePlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestSequencePlanUtilityFunction>();
}

} // namespace alica::tests
