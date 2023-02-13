#include "SimpleTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

SimpleTestPlan::SimpleTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SimpleTestPlan::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
}

std::unique_ptr<SimpleTestPlan> SimpleTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SimpleTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> SimpleTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SimpleTestPlanUtilityFunction> SimpleTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SimpleTestPlanUtilityFunction>();
}

} // namespace alica::tests
