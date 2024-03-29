#include <alica_tests/SimpleSwitches.h>
#include <alica_tests/plans/HandleFailExplicit.h>

namespace alica
{
HandleFailExplicit::HandleFailExplicit(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void HandleFailExplicit::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("aToBSwitch", 0);
    bb.set<int64_t>("cToDSwitch", 2);
}
} // namespace alica
