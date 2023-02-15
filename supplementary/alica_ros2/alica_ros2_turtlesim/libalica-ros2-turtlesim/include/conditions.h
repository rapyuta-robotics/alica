#pragma once

#include <boost/dll/alias.hpp>

namespace alica
{
class Blackboard;
class RunningPlan;

bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);

BOOST_DLL_ALIAS(alica::conditionMove2Init, conditionMove2Init)
BOOST_DLL_ALIAS(alica::conditionInit2Move, conditionInit2Move)
BOOST_DLL_ALIAS(alica::conditionDefaultCondition, conditionDefaultCondition)
} /* namespace alica */
