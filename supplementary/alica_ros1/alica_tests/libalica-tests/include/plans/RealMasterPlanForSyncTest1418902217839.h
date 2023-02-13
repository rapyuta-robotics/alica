#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class RealMasterPlanForSyncTest : public alica::BasicPlan
{
public:
    RealMasterPlanForSyncTest(alica::PlanContext& context);
    static std::unique_ptr<RealMasterPlanForSyncTest> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::RealMasterPlanForSyncTest::create, RealMasterPlanForSyncTest)

class RealMasterPlanForSyncTestUtilityFunction : public alica::BasicUtilityFunction
{
public:
    RealMasterPlanForSyncTestUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<RealMasterPlanForSyncTestUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::RealMasterPlanForSyncTestUtilityFunction::create, RealMasterPlanForSyncTestUtilityFunction)

} // namespace alica::tests
