#pragma once

#include <engine/BasicUtilityFunction.h>
#include <memory>

namespace alica
{
using std::shared_ptr; // TODO: remove when templates can be changed

class IUtilityCreator
{
public:
    virtual ~IUtilityCreator() {}

    virtual std::shared_ptr<BasicUtilityFunction> createUtility(int64_t utilityfunctionConfId, UtilityFunctionContext& context) = 0;
};

} /* namespace alica */
