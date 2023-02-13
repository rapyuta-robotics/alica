#include "SchedulingTestSequenceSubPlan1.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestSequenceSubPlan1::SchedulingTestSequenceSubPlan1(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestSequenceSubPlan1::onInit() {}

std::unique_ptr<SchedulingTestSequenceSubPlan1> SchedulingTestSequenceSubPlan1::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestSequenceSubPlan1>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestSequenceSubPlan1UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestSequenceSubPlan1UtilityFunction> SchedulingTestSequenceSubPlan1UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestSequenceSubPlan1UtilityFunction>();
}

} // namespace alica::tests
