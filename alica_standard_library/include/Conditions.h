#pragma once

#include <boost/dll/alias.hpp>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

namespace alica_standard_library
{

bool AnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AnyChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AllChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AlwaysTrueCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool AlwaysFalseCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool Equals(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool IsChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);

BOOST_DLL_ALIAS(alica_standard_library::AnyChildSuccess, AnyChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AllChildSuccess, AllChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AnyChildFailure, AnyChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AllChildFailure, AllChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysTrueCondition, AlwaysTrueCondition)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysFalseCondition, AlwaysFalseCondition)
BOOST_DLL_ALIAS(alica_standard_library::Equals, Equals)
BOOST_DLL_ALIAS(alica_standard_library::IsChildSuccess, IsChildSuccess)

} /* namespace alica_standard_library */
