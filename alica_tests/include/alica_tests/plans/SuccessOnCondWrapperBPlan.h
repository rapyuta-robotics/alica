#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperBPlan : public BasicPlan
{
public:
    SuccessOnCondWrapperBPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperBPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperBPlanUtilityFunction)

} /* namespace alica */
