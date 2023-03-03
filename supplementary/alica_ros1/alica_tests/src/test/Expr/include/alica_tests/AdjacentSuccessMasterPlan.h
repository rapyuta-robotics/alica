#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessMasterPlan : public BasicPlan
{
public:
    AdjacentSuccessMasterPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessMasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessMasterPlanUtilityFunction)
} /* namespace alica */
