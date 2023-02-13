#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestParameterPassingMaster : public alica::BasicPlan
{
public:
    TestParameterPassingMaster(alica::PlanContext& context);
    static std::unique_ptr<TestParameterPassingMaster> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::TestParameterPassingMaster::create, TestParameterPassingMaster)

class TestParameterPassingMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestParameterPassingMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestParameterPassingMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestParameterPassingMasterUtilityFunction::create, TestParameterPassingMasterUtilityFunction)

} // namespace alica::tests
