#include "TestInheritBlackboardMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestInheritBlackboardMaster::TestInheritBlackboardMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestInheritBlackboardMaster::onInit() {}

std::unique_ptr<TestInheritBlackboardMaster> TestInheritBlackboardMaster::create(alica::PlanContext& context)
{
    return std::make_unique<TestInheritBlackboardMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> TestInheritBlackboardMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestInheritBlackboardMasterUtilityFunction> TestInheritBlackboardMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestInheritBlackboardMasterUtilityFunction>();
}

} // namespace alica::tests
