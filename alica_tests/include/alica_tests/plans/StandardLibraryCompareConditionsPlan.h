#pragma once
#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class StandardLibraryCompareConditionsPlan : public AlicaTestsPlan<StandardLibraryCompareConditionsPlan>
{
public:
    StandardLibraryCompareConditionsPlan(PlanContext& context);

private:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::StandardLibraryCompareConditionsPlan::create, StandardLibraryCompareConditionsPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, StandardLibraryCompareConditionsPlanUtilityFunction)

} /* namespace alica */
