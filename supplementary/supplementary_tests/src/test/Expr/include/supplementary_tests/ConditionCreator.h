#pragma once

#include <engine/IConditionCreator.h>
#include <iostream>
#include <memory>

namespace alica
{
class BasicCondition;

class ConditionCreator : public IConditionCreator
{
public:
    ConditionCreator();
    virtual ~ConditionCreator();
    std::shared_ptr<BasicCondition> createConditions(int64_t conditionConfId) override;
    std::shared_ptr<BasicCondition> createConditions(const ConditionContext& conditionContext) override;
};

} /* namespace alica */
