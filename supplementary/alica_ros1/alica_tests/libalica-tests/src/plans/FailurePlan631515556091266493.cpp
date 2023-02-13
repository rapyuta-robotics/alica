#include "FailurePlan.h"

#include <alica_tests/TestWorldModel.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

FailurePlan::FailurePlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void FailurePlan::onInit()
{
    auto worldModel = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    worldModel->failurePlanInitCalled();
}
std::unique_ptr<FailurePlan> FailurePlan::create(alica::PlanContext& context)
{
    return std::make_unique<FailurePlan>(context);
}

std::shared_ptr<alica::UtilityFunction> FailurePlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<FailurePlanUtilityFunction> FailurePlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<FailurePlanUtilityFunction>();
}

} // namespace alica::tests
