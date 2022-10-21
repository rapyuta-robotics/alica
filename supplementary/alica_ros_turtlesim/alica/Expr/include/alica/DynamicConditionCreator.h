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
    virtual ~DynamicConditionCreator();
    std::shared_ptr<BasicCondition> createConditions(ConditionContext& conditionContext) override;
    std::shared_ptr<BasicCondition> createConditions(int64_t conditionConfId);

private:
    typedef std::shared_ptr<BasicCondition>(conditionCreatorType)(ConditionContext&);
    std::function<conditionCreatorType> _conditionCreator;
};

} /* namespace alica */
