#pragma once

#include <engine/ITransitionConditionCreator.h>
#include <engine/model/Transition.h>

namespace alica
{
class LegacyTransitionConditionCreator : public ITransitionConditionCreator
{
public:
    LegacyTransitionConditionCreator();
    virtual ~LegacyTransitionConditionCreator();

    std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> createConditions(TransitionConditionContext& context);
};
} /* namespace alica */
