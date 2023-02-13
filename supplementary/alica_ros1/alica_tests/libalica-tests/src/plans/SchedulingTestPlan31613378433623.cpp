#include "SchedulingTestPlan3.h"

#include <alica/test/CounterClass.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestPlan3::SchedulingTestPlan3(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestPlan3::onInit()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan3::onTerminate()
{
    CounterClass::called += 1;
}

std::unique_ptr<SchedulingTestPlan3> SchedulingTestPlan3::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestPlan3>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestPlan3UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestPlan3UtilityFunction> SchedulingTestPlan3UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestPlan3UtilityFunction>();
}

} // namespace alica::tests
