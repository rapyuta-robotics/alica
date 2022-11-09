#pragma once

#include "engine/Types.h"

namespace alica
{
class RunningPlan;
class Blackboard;
class TransitionConditionContext;

class ITransitionConditionCreator
{
public:
    virtual ~ITransitionConditionCreator() {}
    virtual TransitionConditionCallback createConditions(TransitionConditionContext& context) = 0;
};

} /* namespace alica */
