#include "Tackle.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

Tackle::Tackle(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void Tackle::onInit() {}

std::unique_ptr<Tackle> Tackle::create(alica::PlanContext& context)
{
    return std::make_unique<Tackle>(context);
}

std::shared_ptr<alica::UtilityFunction> TackleUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TackleUtilityFunction> TackleUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TackleUtilityFunction>();
}

} // namespace alica::tests
