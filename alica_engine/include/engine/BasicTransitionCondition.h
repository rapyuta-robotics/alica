#pragma once

#include "engine/RunnableObject.h"

namespace alica
{

class RunningPlan;
class TransitionCondition;

struct TransitionConditionContext
{
    IAlicaWorldModel* worldModel;
    const std::string name;
    const TransitionCondition* transitionConditionModel;
    std::function<bool(RunningPlan*, Blackboard*)> evalCallback;  
};

class BasicTransitionCondition : private RunnableObject
{
public:
    using RunnableObject::addKeyMapping;
    using RunnableObject::getBlackboard;
    using RunnableObject::getKeyMapping;

    BasicTransitionCondition(TransitionConditionContext& context);
    virtual ~BasicTransitionCondition();
    virtual bool evaluate(RunningPlan* rp);
protected:
    virtual void onInit(){};
    virtual void onTerminate(){};
    virtual void doInit(){};
    virtual void doTerminate(){};

private:
    const TransitionCondition* _transitionCondition;
    std::function<bool(RunningPlan*, Blackboard*)> _evalCallback;
};

} /* namespace alica */
