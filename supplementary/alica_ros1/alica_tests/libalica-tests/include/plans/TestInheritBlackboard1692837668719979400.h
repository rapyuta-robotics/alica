#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestInheritBlackboard : public alica::BasicPlan
{
public:
    TestInheritBlackboard(alica::PlanContext& context);
    static std::unique_ptr<TestInheritBlackboard> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::TestInheritBlackboard::create, TestInheritBlackboard)

class TestInheritBlackboardUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestInheritBlackboardUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestInheritBlackboardUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestInheritBlackboardUtilityFunction::create, TestInheritBlackboardUtilityFunction)

} // namespace alica::tests
