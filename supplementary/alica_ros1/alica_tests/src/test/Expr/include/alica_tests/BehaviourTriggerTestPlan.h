#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehaviourTriggerTestPlan : public BasicPlan
{
public:
    BehaviourTriggerTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehaviourTriggerTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehaviourTriggerTestPlanUtilityFunction)
} /* namespace alica */
