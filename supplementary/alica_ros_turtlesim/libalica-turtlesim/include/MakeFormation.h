#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>

namespace turtlesim
{

BOOST_DLL_ALIAS(alica::BasicPlan::create, MakeFormation)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MakeFormationUtilityFunction)

} /* namespace turtlesim */
