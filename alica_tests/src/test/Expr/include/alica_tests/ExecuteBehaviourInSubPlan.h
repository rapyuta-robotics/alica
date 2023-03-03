#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ExecuteBehaviourInSubPlan : public BasicPlan
{
public:
    ExecuteBehaviourInSubPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ExecuteBehaviourInSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ExecuteBehaviourInSubPlanUtilityFunction)
} /* namespace alica */
