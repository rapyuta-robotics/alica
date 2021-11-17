#pragma once

#include <engine/IUtilityCreator.h>
#include <memory>

namespace alica
{

class UtilityFunctionCreator : public IUtilityCreator
{
public:
    virtual ~UtilityFunctionCreator();
    UtilityFunctionCreator();
    std::shared_ptr<BasicUtilityFunction> createUtility(int64_t utilityfunctionConfId);
};

} /* namespace alica */
