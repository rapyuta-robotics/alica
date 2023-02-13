#include "SchedulingTestPlan1.h"

#include <alica/test/CounterClass.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SchedulingTestPlan1::SchedulingTestPlan1(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SchedulingTestPlan1::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("Plan2Sub", 2);
    bb.set<int64_t>("Init2Term", 5);
    CounterClass::called = 1;
}

void SchedulingTestPlan1::onTerminate()
{
    CounterClass::called += 1;
}

std::unique_ptr<SchedulingTestPlan1> SchedulingTestPlan1::create(alica::PlanContext& context)
{
    return std::make_unique<SchedulingTestPlan1>(context);
}

std::shared_ptr<alica::UtilityFunction> SchedulingTestPlan1UtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SchedulingTestPlan1UtilityFunction> SchedulingTestPlan1UtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SchedulingTestPlan1UtilityFunction>();
}

} // namespace alica::tests
