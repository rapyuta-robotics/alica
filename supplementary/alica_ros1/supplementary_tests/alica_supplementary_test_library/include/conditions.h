#pragma once

#include <boost/dll/alias.hpp>

namespace alica
{
class Blackboard;
class RunningPlan;

bool conditionVariableHandlingStart(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);

BOOST_DLL_ALIAS(alica::conditionVariableHandlingStart, VariableHandlingStart)
BOOST_DLL_ALIAS(alica::conditionDefaultCondition, conditionDefaultCondition)
} /* namespace alica */
