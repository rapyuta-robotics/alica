#pragma once

#include <engine/ITransitionConditionCreator.h>
#include <functional>

namespace alica
{
class TransitionConditionCreator : public ITransitionConditionCreator
{
public:
    TransitionConditionCreator();
    virtual ~TransitionConditionCreator();

    std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> createConditions(int64_t conditionId);
};
} /* namespace alica */
