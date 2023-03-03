#pragma once

#include <engine/BasicUtilityFunction.h>

namespace alica
{

template <class UtilityFunction>
class AlicaTestsUtilityFunction : public alica::BasicUtilityFunction
{
public:
    AlicaTestsUtilityFunction(alica::UtilityFunctionContext& context)
            : alica::BasicUtilityFunction()
    {
    }
    static std::shared_ptr<UtilityFunction> create(alica::UtilityFunctionContext& context) { return std::make_shared<UtilityFunction>(context); }
};

} // namespace alica
