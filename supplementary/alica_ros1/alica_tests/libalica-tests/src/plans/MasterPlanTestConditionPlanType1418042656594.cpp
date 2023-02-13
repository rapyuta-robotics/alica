#include "MasterPlanTestConditionPlanType.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MasterPlanTestConditionPlanType::MasterPlanTestConditionPlanType(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MasterPlanTestConditionPlanType::onInit() {}

std::unique_ptr<MasterPlanTestConditionPlanType> MasterPlanTestConditionPlanType::create(alica::PlanContext& context)
{
    return std::make_unique<MasterPlanTestConditionPlanType>(context);
}

std::shared_ptr<alica::UtilityFunction> MasterPlanTestConditionPlanTypeUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterPlanTestConditionPlanTypeUtilityFunction> MasterPlanTestConditionPlanTypeUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterPlanTestConditionPlanTypeUtilityFunction>();
}

} // namespace alica::tests
