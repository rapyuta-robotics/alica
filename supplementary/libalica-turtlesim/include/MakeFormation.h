#pragma once

#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace turtlesim
{

BOOST_DLL_ALIAS(alica::BasicPlan::create, MakeFormation)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MakeFormationUtilityFunction)

} /* namespace turtlesim */
