#pragma once

#include <memory>
#include <string>

namespace alica
{
class Plan;
class UtilityFunction;

struct UtilityFunctionContext
{
    const std::string name;
    const std::string libraryName;
    int64_t utilityFunctionId;
};

class BasicUtilityFunction
{
public:
    BasicUtilityFunction();
    virtual ~BasicUtilityFunction();

    static std::shared_ptr<BasicUtilityFunction> create(alica::UtilityFunctionContext&);
    virtual std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

} /* namespace alica */
