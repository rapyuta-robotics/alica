#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class Simulation : public alica::BasicPlan
{
public:
    Simulation(alica::PlanContext& context);
    static std::unique_ptr<Simulation> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Simulation::create, Simulation)

class SimulationUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SimulationUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SimulationUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(turtlesim::SimulationUtilityFunction::create, SimulationUtilityFunction)

struct TransitionConditions
{
    static bool CustomAllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
};
BOOST_DLL_ALIAS(turtlesim::TransitionConditions::CustomAllChildSuccess, CustomAllChildSuccess)

} // namespace turtlesim
