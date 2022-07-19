#pragma once

namespace alica
{
class Blackboard;
class RunningPlan;
class IAlicaWorldModel;

bool defaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm);
} /* namespace alica */
