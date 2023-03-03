#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanSuccessTestPlan : public BasicPlan
{
public:
    PlanSuccessTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanSuccessTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanSuccessTestPlanUtilityFunction)
} /* namespace alica */
