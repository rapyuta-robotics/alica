#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MasterSyncTransition : public alica::BasicPlan
{
public:
    MasterSyncTransition(alica::PlanContext& context);
    static std::unique_ptr<MasterSyncTransition> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MasterSyncTransition::create, MasterSyncTransition)

class MasterSyncTransitionUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MasterSyncTransitionUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MasterSyncTransitionUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MasterSyncTransitionUtilityFunction::create, MasterSyncTransitionUtilityFunction)

} // namespace alica::tests
