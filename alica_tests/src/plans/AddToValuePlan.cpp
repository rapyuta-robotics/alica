#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/AddToValuePlan.h>

namespace alica
{
AddToValuePlan::AddToValuePlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void AddToValuePlan::onInit()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("result", bb.get<int64_t>("valueBase") + bb.get<int64_t>("valueAddition"));
}
} // namespace alica
