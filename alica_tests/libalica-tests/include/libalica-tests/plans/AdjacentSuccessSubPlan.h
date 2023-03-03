#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessSubPlan : public BasicPlan
{
public:
    AdjacentSuccessSubPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessSubPlanUtilityFunction)
} /* namespace alica */
