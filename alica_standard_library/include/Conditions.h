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
template <typename T>
bool IsEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
template <typename T>
bool IsNotEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
template <typename T>
bool IsGreaterThan(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
template <typename T>
bool IsLessThan(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
template <typename T>
bool IsLessThanOrEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
template <typename T>
bool IsGreaterThanOrEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);

BOOST_DLL_ALIAS(alica_standard_library::AnyChildSuccess, AnyChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AllChildSuccess, AllChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::AnyChildFailure, AnyChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AllChildFailure, AllChildFailure)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysTrueCondition, AlwaysTrueCondition)
BOOST_DLL_ALIAS(alica_standard_library::AlwaysFalseCondition, AlwaysFalseCondition)
BOOST_DLL_ALIAS(alica_standard_library::IsChildSuccess, IsChildSuccess)
BOOST_DLL_ALIAS(alica_standard_library::IsChildFailure, IsChildFailure)

// IsEqual
BOOST_DLL_ALIAS(alica_standard_library::IsEqual<double>, IsEqualDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsEqual<int64_t>, IsEqualInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsEqual<uint64_t>, IsEqualUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsEqual<std::string>, IsEqualString)
BOOST_DLL_ALIAS(alica_standard_library::IsEqual<bool>, IsEqualBool)

// IsNotEqual
BOOST_DLL_ALIAS(alica_standard_library::IsNotEqual<double>, IsNotEqualDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsNotEqual<int64_t>, IsNotEqualInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsNotEqual<uint64_t>, IsNotEqualUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsNotEqual<std::string>, IsNotEqualString)
BOOST_DLL_ALIAS(alica_standard_library::IsNotEqual<bool>, IsNotEqualBool)

// IsGreaterThan
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThan<int64_t>, IsGreaterThanInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThan<uint64_t>, IsGreaterThanUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThan<double>, IsGreaterThanDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThan<std::string>, IsGreaterThanString)

// IsLessThan
BOOST_DLL_ALIAS(alica_standard_library::IsLessThan<int64_t>, IsLessThanInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThan<uint64_t>, IsLessThanUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThan<double>, IsLessThanDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThan<std::string>, IsLessThanString)

// IsLessThanOrEqual
BOOST_DLL_ALIAS(alica_standard_library::IsLessThanOrEqual<int64_t>, IsLessThanOrEqualInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThanOrEqual<uint64_t>, IsLessThanOrEqualUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThanOrEqual<double>, IsLessThanOrEqualDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsLessThanOrEqual<std::string>, IsLessThanOrEqualString)

// IsGreaterThanOrEqual
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThanOrEqual<int64_t>, IsGreaterThanOrEqualInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThanOrEqual<uint64_t>, IsGreaterThanOrEqualUInt64)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThanOrEqual<double>, IsGreaterThanOrEqualDouble)
BOOST_DLL_ALIAS(alica_standard_library::IsGreaterThanOrEqual<std::string>, IsGreaterThanOrEqualString)

} /* namespace alica_standard_library */
