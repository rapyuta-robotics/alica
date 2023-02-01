#pragma once
#include <engine/IUtilityCreator.h>

#include <functional>
#include <memory>
#include <vector>

namespace alica
{

class BasicUtilityFunction;

class DynamicUtilityFunctionCreator : public IUtilityCreator
{
public:
    DynamicUtilityFunctionCreator();
    virtual ~DynamicUtilityFunctionCreator(){};
    std::shared_ptr<BasicUtilityFunction> createUtility(int64_t conditionConfId, UtilityFunctionContext& utilityFunctionContext) override;

private:
    typedef std::shared_ptr<BasicUtilityFunction>(utilityFunctionCreatorType)(UtilityFunctionContext&);
    std::function<utilityFunctionCreatorType> _utilityFunctionCreator;
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
