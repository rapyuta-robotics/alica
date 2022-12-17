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
    TransitionConditionCallback createConditions(int64_t conditionConfId, TransitionConditionContext& context) override;

private:
    typedef bool(transitionConditionFunctionType)(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*);
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
