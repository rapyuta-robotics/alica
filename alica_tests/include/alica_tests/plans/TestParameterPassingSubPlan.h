#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestParameterPassingSubPlan : public AlicaTestsPlan<TestParameterPassingSubPlan>
{
public:
    TestParameterPassingSubPlan(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::TestParameterPassingSubPlan::create, TestParameterPassingSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassingSubPlanUtilityFunction)

} /* namespace alica */
