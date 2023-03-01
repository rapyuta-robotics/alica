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

struct TransitionConditions
{
    static bool CustomAnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
};
BOOST_DLL_ALIAS(turtlesim::TransitionConditions::CustomAnyChildSuccess, CustomAnyChildSuccess)

} // namespace turtlesim
