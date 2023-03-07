#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicConstraint.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ProblemDescriptor;
class TestPlanKeyMappingPlan : public AlicaTestsPlan<TestPlanKeyMappingPlan>
{
public:
    TestPlanKeyMappingPlan(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestPlanKeyMappingPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestPlanKeyMappingPlanUtilityFunction)
} /* namespace alica */
