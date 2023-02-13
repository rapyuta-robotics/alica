#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestParameterPassing : public alica::BasicPlan
{
public:
    TestParameterPassing(alica::PlanContext& context);
    static std::unique_ptr<TestParameterPassing> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::TestParameterPassing::create, TestParameterPassing)

class TestParameterPassingUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestParameterPassingUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestParameterPassingUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestParameterPassingUtilityFunction::create, TestParameterPassingUtilityFunction)

} // namespace alica::tests
