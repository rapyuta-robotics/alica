#pragma once
#include <engine/ITransitionConditionCreator.h>

#include <functional>
#include <memory>
#include <vector>

namespace alica
{

class BasicCondition;

class DynamicTransitionConditionCreator : public ITransitionConditionCreator
{
public:
    DynamicTransitionConditionCreator();
    virtual ~DynamicTransitionConditionCreator(){};
    TransitionConditionCallback createConditions(TransitionConditionContext& context);

private:
    typedef bool(transitionConditionFunctionType)(const Blackboard*, const RunningPlan*, const Blackboard*);
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
