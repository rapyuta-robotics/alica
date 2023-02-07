#pragma once

#include <boost/dll/alias.hpp>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

namespace utils
{

bool AnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);

BOOST_DLL_ALIAS(utils::AnyChildSuccess, AnyChildSuccess)
BOOST_DLL_ALIAS(utils::AllChildSuccess, AllChildSuccess)

} /* namespace utils */
