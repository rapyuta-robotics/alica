#pragma once

namespace alica
{
class Blackboard;
class RunningPlan;

bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
} /* namespace alica */
