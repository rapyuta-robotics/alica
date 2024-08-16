#include <alica_tests/plans/TestGlobalBlackboardMaster.h>

namespace alica
{
TestGlobalBlackboardMaster::TestGlobalBlackboardMaster(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestGlobalBlackboardMaster::onInit() {}

void TestGlobalBlackboardMaster::run()
{
    std::optional<int64_t> behaviourKeyInMaster;
    {
        LockedBlackboardRW bb(*getBlackboard());
        if (bb.hasValue("behaviourKey")) {
            behaviourKeyInMaster = bb.get<int64_t>("behaviourKey");
        }
    }
    if (behaviourKeyInMaster.has_value()) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("behaviourKeyInMaster", behaviourKeyInMaster.value());
    }
}

} // namespace alica
