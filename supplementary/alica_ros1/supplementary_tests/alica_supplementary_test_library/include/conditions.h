#pragma once

#include <boost/dll/alias.hpp>

namespace alica
{
class Blackboard;
class RunningPlan;
class IAlicaWorldModel;

bool conditionVariableHandlingStart(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);

BOOST_DLL_ALIAS(alica::conditionVariableHandlingStart, VariableHandlingStart)
BOOST_DLL_ALIAS(alica::conditionDefaultCondition, conditionDefaultCondition)
} /* namespace alica */