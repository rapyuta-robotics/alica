#include "FrequencyTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

FrequencyTestPlan::FrequencyTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void FrequencyTestPlan::onInit() {}

std::unique_ptr<FrequencyTestPlan> FrequencyTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<FrequencyTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> FrequencyTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<FrequencyTestPlanUtilityFunction> FrequencyTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<FrequencyTestPlanUtilityFunction>();
}

} // namespace alica::tests
