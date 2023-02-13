#include "RealMasterPlanForSyncTest.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

RealMasterPlanForSyncTest::RealMasterPlanForSyncTest(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void RealMasterPlanForSyncTest::onInit() {}

std::unique_ptr<RealMasterPlanForSyncTest> RealMasterPlanForSyncTest::create(alica::PlanContext& context)
{
    return std::make_unique<RealMasterPlanForSyncTest>(context);
}

std::shared_ptr<alica::UtilityFunction> RealMasterPlanForSyncTestUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<RealMasterPlanForSyncTestUtilityFunction> RealMasterPlanForSyncTestUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<RealMasterPlanForSyncTestUtilityFunction>();
}

} // namespace alica::tests
