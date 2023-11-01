#pragma once
#include <engine/IConditionCreator.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace alica
{

class BasicCondition;

class DynamicConditionCreator : public IConditionCreator
{
public:
    DynamicConditionCreator();
    virtual ~DynamicConditionCreator(){};
    std::shared_ptr<BasicCondition> createConditions(int64_t conditionConfId, ConditionContext& conditionContext) override;

private:
    typedef std::shared_ptr<BasicCondition>(conditionCreatorType)(ConditionContext&);
    std::unordered_map<std::string, std::function<conditionCreatorType>> _conditionCreatorMap; // see DynamicBehaviourCreator for an explanation
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
