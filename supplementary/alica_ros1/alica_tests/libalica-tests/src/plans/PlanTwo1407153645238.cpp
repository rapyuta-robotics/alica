#include "PlanTwo.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanTwo::PlanTwo(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanTwo::onInit() {}

std::unique_ptr<PlanTwo> PlanTwo::create(alica::PlanContext& context)
{
    return std::make_unique<PlanTwo>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanTwoUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanTwoUtilityFunction> PlanTwoUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanTwoUtilityFunction>();
}

} // namespace alica::tests
