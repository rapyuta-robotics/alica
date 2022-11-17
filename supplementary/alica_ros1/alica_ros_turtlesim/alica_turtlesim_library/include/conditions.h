#pragma once
#include <boost/dll/alias.hpp>

namespace alica
{
class Blackboard;
class RunningPlan;
class IAlicaWorldModel;

bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);

BOOST_DLL_ALIAS(alica::conditionMove2Init, Move2Init)
BOOST_DLL_ALIAS(alica::conditionInit2Move, Init2Move)
BOOST_DLL_ALIAS(alica::conditionDefaultCondition, DefaultCondition)

} /* namespace alica */
