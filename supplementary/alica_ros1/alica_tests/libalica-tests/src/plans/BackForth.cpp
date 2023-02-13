#include "BackForth.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

BackForth::BackForth(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void BackForth::onInit() {}

std::unique_ptr<BackForth> BackForth::create(alica::PlanContext& context)
{
    return std::make_unique<BackForth>(context);
}

std::shared_ptr<alica::UtilityFunction> BackForthUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<BackForthUtilityFunction> BackForthUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<BackForthUtilityFunction>();
}

} // namespace alica::tests
