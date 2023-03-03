#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ReadConfigurationPlan : public BasicPlan
{
public:
    ReadConfigurationPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ReadConfigurationPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ReadConfigurationPlanUtilityFunction)
} /* namespace alica */
