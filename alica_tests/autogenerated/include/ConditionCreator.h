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
    std::shared_ptr<BasicCondition> createConditions(long conditionConfId);
};

} /* namespace alica */
