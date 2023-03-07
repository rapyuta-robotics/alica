#pragma once

#include <engine/BasicCondition.h>

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestBlackboardPlan : public AlicaTestsPlan<TestBlackboardPlan>
{
public:
    TestBlackboardPlan(PlanContext& context);
};

bool ValueMappingConditionTestState2BlackboardTestSuccessState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);

BOOST_DLL_ALIAS(alica::TestBlackboardPlan::create, TestBlackboardPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestBlackboardPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::ValueMappingConditionTestState2BlackboardTestSuccessState, ValueMappingConditionTestState2BlackboardTestSuccessState)
} /* namespace alica */
