#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

BOOST_DLL_ALIAS(alica::BasicPlan::create, SurroundLeaderTutorial)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SurroundLeaderTutorialUtilityFunction)

} // namespace turtlesim
