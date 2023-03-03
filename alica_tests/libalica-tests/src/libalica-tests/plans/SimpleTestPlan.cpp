#include <alica/test/CounterClass.h>
#include <libalica-tests/plans/SimpleTestPlan.h>

namespace alica
{
SimpleTestPlan::SimpleTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool SimpleTestPlanPreCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool SimpleTestPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    CounterClass::called++;
    return true;
}

void SimpleTestPlan::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
}
} // namespace alica
