#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestParameterPassingPlan : public AlicaTestsPlan<TestParameterPassingPlan>
{
public:
    TestParameterPassingPlan(PlanContext& context);

private:
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::TestParameterPassingPlan::create, TestParameterPassingPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassingPlanUtilityFunction)

} /* namespace alica */
