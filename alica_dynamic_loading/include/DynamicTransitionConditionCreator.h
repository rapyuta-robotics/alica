#pragma once
#include <engine/ITransitionConditionCreator.h>

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace alica
{

class BasicCondition;

class DynamicTransitionConditionCreator : public ITransitionConditionCreator
{
public:
    DynamicTransitionConditionCreator();
    virtual ~DynamicTransitionConditionCreator(){};
    TransitionConditionCallback createConditions(int64_t conditionId, TransitionConditionContext& context);

private:
    typedef bool(transitionConditionFunctionType)(const Blackboard*, const RunningPlan*, const Blackboard*);
    std::unordered_map<std::string, TransitionConditionCallback> _transitionConditionMap; // See DynamicBehaviourCreator for an explanation
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
