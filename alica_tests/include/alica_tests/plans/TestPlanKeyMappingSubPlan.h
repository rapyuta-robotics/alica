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
class TestPlanKeyMappingSubPlan : public AlicaTestsPlan<TestPlanKeyMappingSubPlan>
{
public:
    TestPlanKeyMappingSubPlan(PlanContext& context);
};

bool TestPlanKeyMappingSubPlanEntryState2SuccessState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestPlanKeyMappingSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestPlanKeyMappingSubPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::TestPlanKeyMappingSubPlanEntryState2SuccessState, TestPlanKeyMappingSubPlanEntryState2SuccessState)
} /* namespace alica */
