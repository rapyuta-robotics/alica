#pragma once
#include <engine/IConditionCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicCondition;

class DynamicConditionCreator : public IConditionCreator
{
public:
    DynamicConditionCreator(const std::string& defaultLibraryPath);
    virtual ~DynamicConditionCreator();
    std::shared_ptr<BasicCondition> createConditions(ConditionContext& conditionContext) override;

private:
    std::string _defaultLibraryPath;
};

} /* namespace alica */
