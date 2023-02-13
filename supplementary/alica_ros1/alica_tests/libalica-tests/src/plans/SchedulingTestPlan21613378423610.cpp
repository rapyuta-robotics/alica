#include "SchedulingTestPlan2.h"

#include <alica/test/CounterClass.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestPlan2::SchedulingTestPlan2(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestPlan2::onInit()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan2::onTerminate()
{
    CounterClass::called += 1;
}

std::unique_ptr<SchedulingTestPlan2> SchedulingTestPlan2::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestPlan2>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestPlan2UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestPlan2UtilityFunction> SchedulingTestPlan2UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestPlan2UtilityFunction>();
}

} // namespace alica::tests
