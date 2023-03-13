#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>

namespace alica
{
class IsChildSuccessTestPlan : public BasicPlan
{
public:
    IsChildSuccessTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, IsChildSuccessTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, IsChildSuccessTestPlanUtilityFunction)
} /* namespace alica */
