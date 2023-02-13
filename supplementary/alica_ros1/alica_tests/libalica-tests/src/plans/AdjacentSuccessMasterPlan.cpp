#include "AdjacentSuccessMasterPlan.h"

#include <alica_tests/TestWorldModel.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

AdjacentSuccessMasterPlan::AdjacentSuccessMasterPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void AdjacentSuccessMasterPlan::onInit() {}

std::unique_ptr<AdjacentSuccessMasterPlan> AdjacentSuccessMasterPlan::create(alica::PlanContext& context)
{
    return std::make_unique<AdjacentSuccessMasterPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> AdjacentSuccessMasterPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<AdjacentSuccessMasterPlanUtilityFunction> AdjacentSuccessMasterPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<AdjacentSuccessMasterPlanUtilityFunction>();
}

} // namespace alica::tests
