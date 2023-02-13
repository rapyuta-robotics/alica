#include "PlanThree.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanThree::PlanThree(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanThree::onInit() {}

std::unique_ptr<PlanThree> PlanThree::create(alica::PlanContext& context)
{
    return std::make_unique<PlanThree>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanThreeUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanThreeUtilityFunction> PlanThreeUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanThreeUtilityFunction>();
}

} // namespace alica::tests
