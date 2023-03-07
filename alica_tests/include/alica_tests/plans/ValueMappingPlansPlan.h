#pragma once

#include <engine/BasicCondition.h>

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ValueMappingPlansPlan : public AlicaTestsPlan<ValueMappingPlansPlan>
{
public:
    ValueMappingPlansPlan(PlanContext& context);
};

bool ValueMappingPlansState2ValueMappingPlansSuccessState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);

BOOST_DLL_ALIAS(alica::ValueMappingPlansPlan::create, ValueMappingPlansPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ValueMappingPlansPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::ValueMappingPlansState2ValueMappingPlansSuccessState, ValueMappingPlansState2ValueMappingPlansSuccessState)
} /* namespace alica */
