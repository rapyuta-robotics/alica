#include "TestParameterPassingMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestParameterPassingMaster::TestParameterPassingMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestParameterPassingMaster::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("masterKey", 8);
}

std::unique_ptr<TestParameterPassingMaster> TestParameterPassingMaster::create(alica::PlanContext& context)
{
    return std::make_unique<TestParameterPassingMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> TestParameterPassingMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestParameterPassingMasterUtilityFunction> TestParameterPassingMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestParameterPassingMasterUtilityFunction>();
}

} // namespace alica::tests
