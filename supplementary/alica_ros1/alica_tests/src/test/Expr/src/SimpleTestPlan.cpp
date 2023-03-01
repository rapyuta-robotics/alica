#include <alica/test/CounterClass.h>
#include <alica_tests/SimpleTestPlan.h>

namespace alica
{
SimpleTestPlan::SimpleTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool PreCondition1412781707952::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool RunTimeCondition1412781693884::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
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
