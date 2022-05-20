#pragma once

#include <functional>

namespace alica
{
class RunningPlan;
class Blackboard;

class ITransitionPreConditionCreator
{
public:
    virtual ~ITransitionPreConditionCreator() {}
    virtual std::function<bool(RunningPlan*, Blackboard*)> createConditions(int64_t conditionId) = 0;
};

} /* namespace alica */
