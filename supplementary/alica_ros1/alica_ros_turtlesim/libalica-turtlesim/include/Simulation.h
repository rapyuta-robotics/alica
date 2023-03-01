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
    void onInit() override;
    static std::unique_ptr<Simulation> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Simulation::create, Simulation)

BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SimulationUtilityFunction)

struct TransitionConditions
{
    static bool CustomAllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
};
BOOST_DLL_ALIAS(turtlesim::TransitionConditions::CustomAllChildSuccess, CustomAllChildSuccess)

} // namespace turtlesim
