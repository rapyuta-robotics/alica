#include <alica_tests/plans/TestStandardLibraryCompareConditions.h>

namespace alica
{
TestStandardLibraryCompareConditions::TestStandardLibraryCompareConditions(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestStandardLibraryCompareConditions::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<double>("valueDouble", 3.15);
    bb.set<int64_t>("valueInt64", -4560);
    bb.set<std::string>("valueString", "test_equal_string");
    bb.set<uint64_t>("valueUInt64", 1230);
}
} // namespace alica
