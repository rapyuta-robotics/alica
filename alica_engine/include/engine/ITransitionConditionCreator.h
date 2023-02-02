#pragma once

#include "engine/Types.h"

namespace alica
{
class RunningPlan;
class Blackboard;
struct TransitionConditionContext;

class ITransitionConditionCreator
{
public:
    virtual ~ITransitionConditionCreator() {}
    virtual TransitionConditionCallback createConditions(int64_t conditionId, TransitionConditionContext& context) = 0;
};

} /* namespace alica */
