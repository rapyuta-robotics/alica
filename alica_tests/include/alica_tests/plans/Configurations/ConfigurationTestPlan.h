#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ConfigurationTestPlan : public BasicPlan
{
public:
    ConfigurationTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConfigurationTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConfigurationTestPlanUtilityFunction)
} /* namespace alica */
