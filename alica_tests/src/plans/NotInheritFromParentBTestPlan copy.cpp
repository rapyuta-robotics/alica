#include <alica_tests/plans/NotInheritFromParentBTestPlan.h>
#include <string>

namespace alica
{
NotInheritFromParentBTestPlan::NotInheritFromParentBTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void NotInheritFromParentBTestPlan::onInit()
{
    LockedBlackboardRW bb(*getBlackboard());
    if (bb.hasValue("boolValueA") || bb.hasValue("stringValueA") || bb.hasValue("intValueA") || bb.hasValue("uintValueA") || bb.hasValue("doubleValueA")) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Child NotInheritFromParentBTestPlan inherits blackboard from parent InheritFromParentATestPlan, but should not inherit!");
    } else {
        bb.set("boolValueB", false);
        bb.set("doubleValueB", 3.0);
        bb.set("intValueB", 3);
        bb.set("uintValueB", 3u);
        bb.set("stringValueB", std::string("stringFromB"));
    }
}
} // namespace alica
