#include "SchedulingTestSequenceSubPlan3.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestSequenceSubPlan3::SchedulingTestSequenceSubPlan3(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestSequenceSubPlan3::onInit() {}

std::unique_ptr<SchedulingTestSequenceSubPlan3> SchedulingTestSequenceSubPlan3::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestSequenceSubPlan3>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestSequenceSubPlan3UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestSequenceSubPlan3UtilityFunction> SchedulingTestSequenceSubPlan3UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestSequenceSubPlan3UtilityFunction>();
}

} // namespace alica::tests
