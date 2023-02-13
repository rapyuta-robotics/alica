#include "PlanFour.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanFour::PlanFour(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanFour::onInit() {}

std::unique_ptr<PlanFour> PlanFour::create(alica::PlanContext& context)
{
    return std::make_unique<PlanFour>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanFourUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanFourUtilityFunction> PlanFourUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanFourUtilityFunction>();
}

} // namespace alica::tests
