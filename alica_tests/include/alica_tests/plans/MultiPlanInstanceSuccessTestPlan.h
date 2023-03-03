#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiPlanInstanceSuccessTestPlan : public BasicPlan
{
public:
    MultiPlanInstanceSuccessTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiPlanInstanceSuccessTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiPlanInstanceSuccessTestPlanUtilityFunction)
} /* namespace alica */
