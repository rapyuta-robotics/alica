#include "PlanFive.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanFive::PlanFive(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanFive::onInit() {}

std::unique_ptr<PlanFive> PlanFive::create(alica::PlanContext& context)
{
    return std::make_unique<PlanFive>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanFiveUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanFiveUtilityFunction> PlanFiveUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanFiveUtilityFunction>();
}

} // namespace alica::tests
