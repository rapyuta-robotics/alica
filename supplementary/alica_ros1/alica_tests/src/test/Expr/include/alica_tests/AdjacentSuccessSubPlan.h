#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessSubPlan : public DomainPlan
{
public:
    AdjacentSuccessSubPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessSubPlanUtilityFunction)
} /* namespace alica */
