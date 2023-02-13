#include "TestParameterPassing.h"

#include "alica_tests/TestWorldModel.h"
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestParameterPassing::TestParameterPassing(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestParameterPassing::onInit()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
    bb.set<int64_t>("planKey", 1);
    wm->passedParameters["planKey"] = bb.get<int64_t>("planKey");
    bb.set<int64_t>("planOutputKey", 5);
    bb.set<int64_t>("planSecondOutputKey", 7);
    bb.set<int64_t>("planInputKey", 1);
    wm->passedParameters["planInputFromMaster"] = bb.get<int64_t>("planInputFromMaster");
}

std::unique_ptr<TestParameterPassing> TestParameterPassing::create(alica::PlanContext& context)
{
    return std::make_unique<TestParameterPassing>(context);
}

std::shared_ptr<alica::UtilityFunction> TestParameterPassingUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestParameterPassingUtilityFunction> TestParameterPassingUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestParameterPassingUtilityFunction>();
}

} // namespace alica::tests
