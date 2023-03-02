#include <alica_tests/FailurePlan.h>
#include <alica_tests/TestWorldModel.h>

namespace alica
{
FailurePlan::FailurePlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void FailurePlan::onInit()
{
    auto worldModel = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    worldModel->failurePlanInitCalled();
}
} // namespace alica