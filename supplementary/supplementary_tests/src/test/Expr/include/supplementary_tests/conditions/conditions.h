#pragma once

/*PROTECTED REGION ID(conditionHeader) ENABLED START*/
extern bool vhStartCondition;
/*PROTECTED REGION END*/

namespace alica
{
class Blackboard;
class RunningPlan;
class IAlicaWorldModel;

bool conditionVariableHandlingStart295816226925111421(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
bool conditionDefaultCondition2011598442725310989(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
} /* namespace alica */
