#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MultiAgentTestMaster : public alica::BasicPlan
{
public:
    MultiAgentTestMaster(alica::PlanContext& context);
    static std::unique_ptr<MultiAgentTestMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MultiAgentTestMaster::create, MultiAgentTestMaster)

class MultiAgentTestMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MultiAgentTestMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MultiAgentTestMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MultiAgentTestMasterUtilityFunction::create, MultiAgentTestMasterUtilityFunction)

} // namespace alica::tests
