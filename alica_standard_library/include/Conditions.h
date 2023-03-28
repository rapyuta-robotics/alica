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
bool IsChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool IsEqualDouble(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
bool IsEqualInt64(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsEqualUInt64(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsEqualString(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsEqualBool(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);

BOOST_DLL_ALIAS(alica_standard_library::AnyChildSuccess, AnyChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AllChildSuccess, AllChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AnyChildFailure, AnyChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AllChildFailure, AllChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysTrueCondition, AlwaysTrueCondition)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysFalseCondition, AlwaysFalseCondition)
BOOST_DLL_ALIAS(alica_standard_library::IsChildSuccess, IsChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::IsEqualDouble, IsEqualDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsEqualInt64, IsEqualInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsEqualUInt64, IsEqualUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsEqualString, IsEqualString)
BOOST_DLL_ALIAS(alica_standard_library::IsEqualBool, IsEqualBool)

} /* namespace alica_standard_library */
