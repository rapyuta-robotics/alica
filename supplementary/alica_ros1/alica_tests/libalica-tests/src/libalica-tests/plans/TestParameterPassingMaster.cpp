#include <libalica-tests/plans/TestParameterPassingMaster.h>

namespace alica
{
TestParameterPassingMaster::TestParameterPassingMaster(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestParameterPassingMaster::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("masterKey", 8);
}
} // namespace alica
