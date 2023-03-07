#include <alica_tests/plans/TestParameterPassingPlan.h>

namespace alica
{
TestParameterPassingPlan::TestParameterPassingPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestParameterPassingPlan::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("masterKey", 8);
}
} // namespace alica
