#include "MasterPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MasterPlan::MasterPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MasterPlan::onInit() {}

std::unique_ptr<MasterPlan> MasterPlan::create(alica::PlanContext& context)
{
    return std::make_unique<MasterPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> MasterPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterPlanUtilityFunction> MasterPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterPlanUtilityFunction>();
}

} // namespace alica::tests
