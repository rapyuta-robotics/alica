#pragma once
#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestStandardLibraryCompareConditions : public AlicaTestsPlan<TestStandardLibraryCompareConditions>
{
public:
    TestStandardLibraryCompareConditions(PlanContext& context);

private:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::TestStandardLibraryCompareConditions::create, TestStandardLibraryCompareConditions)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestStandardLibraryCompareConditionsUtilityFunction)

} /* namespace alica */
