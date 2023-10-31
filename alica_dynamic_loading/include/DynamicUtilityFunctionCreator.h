#pragma once
#include <engine/IUtilityCreator.h>

#include <functional>
#include <memory>
#include <unordered_map>
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
    std::unordered_map<std::string, std::function<utilityFunctionCreatorType>> _utilityFunctionCreatorMap; // See DynamicBehaviourCreator for an explanation
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
