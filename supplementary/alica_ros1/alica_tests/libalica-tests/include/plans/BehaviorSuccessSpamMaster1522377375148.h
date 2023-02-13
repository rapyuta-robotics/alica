#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class BehaviourSuccessSpamMaster : public alica::BasicPlan
{
public:
    BehaviourSuccessSpamMaster(alica::PlanContext& context);
    static std::unique_ptr<BehaviourSuccessSpamMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::BehaviourSuccessSpamMaster::create, BehaviourSuccessSpamMaster)

class BehaviourSuccessSpamMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    BehaviourSuccessSpamMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<BehaviourSuccessSpamMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::BehaviourSuccessSpamMasterUtilityFunction::create, BehaviourSuccessSpamMasterUtilityFunction)

} // namespace alica::tests
