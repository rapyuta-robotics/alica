#pragma once

#include <boost/dll/alias.hpp>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

namespace utils
{

bool AnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AnyChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AllChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AlwaysTrueCond(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool DefaultCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);

bool IsAnyChildStatusSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildTaskSuccessfull(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildStatusFailed(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildStatus(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);

BOOST_DLL_ALIAS(utils::AnyChildSuccess, AnyChildSuccess)
BOOST_DLL_ALIAS(utils::AllChildSuccess, AllChildSuccess)
BOOST_DLL_ALIAS(utils::AnyChildFailure, AnyChildFailure)
BOOST_DLL_ALIAS(utils::AllChildFailure, AllChildFailure)
BOOST_DLL_ALIAS(utils::AlwaysTrueCond, AlwaysTrueCond)
BOOST_DLL_ALIAS(utils::DefaultCondition, DefaultCondition)

BOOST_DLL_ALIAS(utils::IsAnyChildStatusSuccess, IsAnyChildStatusSuccess)
BOOST_DLL_ALIAS(utils::IsAnyChildTaskSuccessfull, IsAnyChildTaskSuccessfull)
BOOST_DLL_ALIAS(utils::IsAnyChildStatusFailed, IsAnyChildStatusFailed)
BOOST_DLL_ALIAS(utils::IsAnyChildStatus, IsAnyChildStatus)

} /* namespace utils */
