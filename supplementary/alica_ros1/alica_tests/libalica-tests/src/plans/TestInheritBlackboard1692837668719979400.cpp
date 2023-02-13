#include "TestInheritBlackboard.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

TestInheritBlackboard::TestInheritBlackboard(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TestInheritBlackboard::onInit() {}

std::unique_ptr<TestInheritBlackboard> TestInheritBlackboard::create(alica::PlanContext& context)
{
    return std::make_unique<TestInheritBlackboard>(context);
}

std::shared_ptr<alica::UtilityFunction> TestInheritBlackboardUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<TestInheritBlackboardUtilityFunction> TestInheritBlackboardUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<TestInheritBlackboardUtilityFunction>();
}

} // namespace alica::tests
