#include "PlanOne.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanOne::PlanOne(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanOne::onInit() {}

std::unique_ptr<PlanOne> PlanOne::create(alica::PlanContext& context)
{
    return std::make_unique<PlanOne>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanOneUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanOneUtilityFunction> PlanOneUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanOneUtilityFunction>();
}

} // namespace alica::tests
