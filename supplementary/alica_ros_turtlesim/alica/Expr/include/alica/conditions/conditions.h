#pragma once

/*PROTECTED REGION ID(conditionHeader) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
class Blackboard;
class RunningPlan;
class IAlicaWorldModel;

bool conditionMove2Init748720375848597116(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionInit2Move974606107671315045(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionDefaultCondition2190266318562141841(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
} /* namespace alica */