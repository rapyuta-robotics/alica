#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnInitPlan : public BasicPlan
{
public:
    SuccessOnInitPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnInitPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnInitPlanUtilityFunction)
} /* namespace alica */
