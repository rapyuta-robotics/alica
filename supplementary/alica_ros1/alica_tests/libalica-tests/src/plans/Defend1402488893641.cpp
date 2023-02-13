#include "Defend.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

Defend::Defend(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void Defend::onInit() {}

std::unique_ptr<Defend> Defend::create(alica::PlanContext& context)
{
    return std::make_unique<Defend>(context);
}

std::shared_ptr<alica::UtilityFunction> DefendUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<DefendUtilityFunction> DefendUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<DefendUtilityFunction>();
}

} // namespace alica::tests
