#pragma once
#include <engine/IConditionCreator.h>

#include <functional>
#include <memory>

namespace alica
{

class BasicCondition;

class DynamicConditionCreator : public IConditionCreator
{
public:
    DynamicConditionCreator();
    virtual ~DynamicConditionCreator(){};
    std::shared_ptr<BasicCondition> createConditions(ConditionContext& conditionContext) override;

private:
    typedef std::shared_ptr<BasicCondition>(conditionCreatorType)(ConditionContext&);
    std::function<conditionCreatorType> _conditionCreator;
    std::string _libraryPath;
};

} /* namespace alica */
