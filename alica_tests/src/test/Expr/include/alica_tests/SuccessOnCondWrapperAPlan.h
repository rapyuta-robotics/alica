#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperAPlan : public BasicPlan
{
public:
    SuccessOnCondWrapperAPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperAPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperAPlanUtilityFunction)

} /* namespace alica */
