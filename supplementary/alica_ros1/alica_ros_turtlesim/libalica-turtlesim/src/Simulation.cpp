#include "Simulation.h"

#include <engine/DefaultUtilityFunction.h>

#include "ros/ros.h"

namespace turtlesim
{

Simulation::Simulation(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void Simulation::onInit()
{
    alica::LockedBlackboardRW bb{*getBlackboard()};
    bb.set("join_formation_topic", std::string{"join_formation"});
    bb.set("leave_formation_topic", std::string{"leave_formation"});
}

std::unique_ptr<Simulation> Simulation::create(alica::PlanContext& context)
{
    return std::make_unique<Simulation>(context);
}

namespace
{

bool isSuccess(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Success;
    } else {
        return rp->getActiveState()->isSuccessState();
    }
}

} // namespace

bool TransitionConditions::CustomAllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (!isSuccess(child)) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (rp->getActiveTriple().state) {
        return rp->getChildren().size() >= rp->getActiveTriple().state->getConfAbstractPlanWrappers().size();
    }
    return true;
}

} // namespace turtlesim
