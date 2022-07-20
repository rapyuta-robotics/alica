#pragma once

#include "engine/Types.h"

namespace alica
{
class RunningPlan;
class Blackboard;

class ITransitionConditionCreator
{
public:
    virtual ~ITransitionConditionCreator() {}
    virtual TransitionConditionCallback createConditions(int64_t conditionId) = 0;
};

} /* namespace alica */
