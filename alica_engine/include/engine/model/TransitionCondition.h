#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <memory>
#include <string>
#include <functional>

namespace alica
{
class AbstractPlan;
class ModelFactory;
class TransitionConditionFactory;
class BlackboardBlueprint;
class KeyMapping;
class IAlicaWorldModel;
class RunningPlan;

struct TransitionConditionContext
{
    std::unique_ptr<BlackboardBlueprint> blackboardBlueprint;
};

class TransitionCondition : public AlicaElement
{
public:
    TransitionCondition(TransitionConditionContext& context);
    virtual ~TransitionCondition() = default;
    virtual bool evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping);
    void setEvalCallback(TransitionConditionCallback cb) { _evalCallback = cb; };
private:
    // TODO: Check if friends are necessary
    friend ModelFactory;
    friend TransitionConditionFactory;

    std::unique_ptr<Blackboard> _blackboard;
    TransitionConditionCallback _evalCallback;
};
} // namespace alica
