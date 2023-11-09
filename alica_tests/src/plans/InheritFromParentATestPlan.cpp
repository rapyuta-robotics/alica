#include <alica_tests/plans/InheritFromParentATestPlan.h>
#include <string>

namespace alica
{
InheritFromParentATestPlan::InheritFromParentATestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void InheritFromParentATestPlan::onTerminate()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("boolValueA", true);
    bb.set("doubleValueA", 2.0);
    bb.set("intValueA", 2);
    bb.set("stringValueA", std::string("setString"));
    bb.set("uintValueA", 2u);
}
} // namespace alica
