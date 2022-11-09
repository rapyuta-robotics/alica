#pragma once

#include <engine/ITransitionConditionCreator.h>

namespace alica
{
class TransitionConditionCreator : public ITransitionConditionCreator
{
public:
    TransitionConditionCreator();
    virtual ~TransitionConditionCreator();

    std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> createConditions(TransitionConditionContext& context);
};
} /* namespace alica */
