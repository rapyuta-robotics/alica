#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class Master : public alica::BasicPlan
{
public:
    Master(alica::PlanContext& context);
    static std::unique_ptr<Master> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Master::create, Master)

class MasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(turtlesim::MasterUtilityFunction::create, MasterUtilityFunction)

struct TransitionConditions
{
    static bool CustomAnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
};
BOOST_DLL_ALIAS(turtlesim::TransitionConditions::CustomAnyChildSuccess, CustomAnyChildSuccess)

} // namespace turtlesim
