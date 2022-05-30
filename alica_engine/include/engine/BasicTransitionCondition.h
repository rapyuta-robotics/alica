#pragma once

#include "engine/Types.h"

#include <string>
#include <functional>
#include <memory>

namespace alica
{

class RunningPlan;
class TransitionCondition;
class IAlicaWorldModel;
class Blackboard;
class KeyMapping;

struct TransitionConditionContext
{
    const TransitionCondition* transitionConditionModel;
    TransitionConditionCallback evalCallback;  
};

class BasicTransitionCondition
{
public:
    BasicTransitionCondition(TransitionConditionContext& context);
    virtual ~BasicTransitionCondition();
    virtual bool evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping);

private:
    const TransitionCondition* _transitionCondition;
    TransitionConditionCallback _evalCallback;
    std::unique_ptr<Blackboard> _blackboard;
};

} /* namespace alica */
