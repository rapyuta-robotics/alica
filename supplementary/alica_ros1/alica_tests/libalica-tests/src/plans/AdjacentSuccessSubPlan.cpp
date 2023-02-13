#include "AdjacentSuccessSubPlan.h"
#include <alica_tests/TestWorldModel.h>

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

AdjacentSuccessSubPlan::AdjacentSuccessSubPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void AdjacentSuccessSubPlan::onInit() {}

std::unique_ptr<AdjacentSuccessSubPlan> AdjacentSuccessSubPlan::create(alica::PlanContext& context)
{
    return std::make_unique<AdjacentSuccessSubPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> AdjacentSuccessSubPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<AdjacentSuccessSubPlanUtilityFunction> AdjacentSuccessSubPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<AdjacentSuccessSubPlanUtilityFunction>();
}

} // namespace alica::tests
