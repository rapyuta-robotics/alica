#include <alica_tests/plans/StandardLibraryCompareConditionsPlan.h>

namespace alica
{
StandardLibraryCompareConditionsPlan::StandardLibraryCompareConditionsPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void StandardLibraryCompareConditionsPlan::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<double>("valueDouble", 3.15);
    bb.set<int64_t>("valueInt64", 1234);
    bb.set<std::string>("valueString", "xyz");
    bb.set<uint64_t>("valueUint64", 1234);
    bb.set<bool>("valueBool", true);
}
} // namespace alica
