#pragma once
#include <engine/IConditionCreator.h>

#include <functional>
#include <memory>
#include <string>

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
    const std::string _libraryRelativePath{"/../../../lib/"};
    std::string _currentLibraryPath;

    typedef std::shared_ptr<BasicCondition>(conditionCreatorType)(ConditionContext&);
    std::function<conditionCreatorType> _conditionCreator;
};

} /* namespace alica */
