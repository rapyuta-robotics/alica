#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondPlan : public BasicPlan
{
public:
    SuccessOnCondPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondPlanUtilityFunction)

} /* namespace alica */
