#include "SchedulingTestSequenceSubPlan2.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestSequenceSubPlan2::SchedulingTestSequenceSubPlan2(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestSequenceSubPlan2::onInit() {}

std::unique_ptr<SchedulingTestSequenceSubPlan2> SchedulingTestSequenceSubPlan2::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestSequenceSubPlan2>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestSequenceSubPlan2UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestSequenceSubPlan2UtilityFunction> SchedulingTestSequenceSubPlan2UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestSequenceSubPlan2UtilityFunction>();
}

} // namespace alica::tests
