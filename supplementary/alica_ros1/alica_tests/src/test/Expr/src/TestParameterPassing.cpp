#include "alica_tests/TestWorldModel.h"
#include <alica_tests/TestParameterPassing.h>

namespace alica
{
TestParameterPassing::TestParameterPassing(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestParameterPassing::onInit()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
    bb.set<int64_t>("planKey", 1);
    wm->passedParameters["planKey"] = bb.get<int64_t>("planKey");
    bb.set<int64_t>("planOutputKey", 5);
    bb.set<int64_t>("planSecondOutputKey", 7);
    bb.set<int64_t>("planInputKey", 1);
    wm->passedParameters["planInputFromMaster"] = bb.get<int64_t>("planInputFromMaster");
}
} // namespace alica
