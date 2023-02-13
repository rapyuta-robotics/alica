#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestInheritBlackboardMaster : public alica::BasicPlan
{
public:
    TestInheritBlackboardMaster(alica::PlanContext& context);
    static std::unique_ptr<TestInheritBlackboardMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::TestInheritBlackboardMaster::create, TestInheritBlackboardMaster)

class TestInheritBlackboardMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestInheritBlackboardMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestInheritBlackboardMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestInheritBlackboardMasterUtilityFunction::create, TestInheritBlackboardMasterUtilityFunction)

} // namespace alica::tests
