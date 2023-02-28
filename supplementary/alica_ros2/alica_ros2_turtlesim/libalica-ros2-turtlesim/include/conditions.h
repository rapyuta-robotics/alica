#pragma once

#include <boost/dll/alias.hpp>
namespace alica
{
class Blackboard;
class RunningPlan;
} // namespace alica

namespace turtlesim
{

bool conditionMove2Init(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool conditionInit2Move(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool conditionDefaultCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);

BOOST_DLL_ALIAS(turtlesim::conditionMove2Init, Move2Init)
BOOST_DLL_ALIAS(turtlesim::conditionInit2Move, Init2Move)
BOOST_DLL_ALIAS(turtlesim::conditionDefaultCondition, DefaultCondition)
} /* namespace turtlesim */
