#include "alica/test/Util.h"

#include <engine/Types.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>

namespace alica::test
{
std::shared_ptr<BasicBehaviour> Util::getBasicBehaviour(alica::AlicaEngine* ae, int64_t behaviourID, int64_t configurationID)
{
    std::shared_ptr<alica::BasicBehaviour> behaviour = nullptr;
    for (auto& behaviourEntry : ae->getBehaviourPool().getAvailableBehaviours()) {
        if (behaviourEntry.first->getAbstractPlan()->getId() == behaviourID &&
                (configurationID == 0 ? behaviourEntry.first->getConfiguration() == nullptr
                                      : behaviourEntry.first->getConfiguration()->getId() == configurationID)) {
            behaviour = behaviourEntry.second;
            break;
        }
    }
    return behaviour;
}

BasicPlan* Util::getBasicPlan(alica::AlicaEngine* ae, int64_t planId, int64_t configurationId)
{
    BasicPlan* plan = nullptr;
    for (auto& planEntry : ae->getPlanPool().getAvailablePlans()) {
        if (planEntry.first->getAbstractPlan()->getId() == planId &&
                (configurationId == 0 ? planEntry.first->getConfiguration() == nullptr : planEntry.first->getConfiguration()->getId() == configurationId)) {
            plan = planEntry.second.get();
            break;
        }
    }
    return plan;
}

bool Util::isStateActive(alica::AlicaEngine* ae, int64_t id)
{
    return isStateActiveHelper(ae->getPlanBase().getRootNode(), id);
}

bool Util::isStateActiveHelper(const RunningPlan* rp, int64_t id)
{
    if (!rp) {
        return false;
    }

    const State* activeState = rp->getActiveState();
    if (activeState && activeState->getId() == id) {
        return true;
    }

    for (const auto& child : rp->getChildren()) {
        if (isStateActiveHelper(child, id)) {
            return true;
        }
    }
    return false;
}

bool Util::isPlanActive(alica::AlicaEngine* ae, int64_t id)
{
    return isPlanActiveHelper(ae->getPlanBase().getRootNode(), id);
}

bool Util::isPlanActiveHelper(const RunningPlan* rp, int64_t id)
{
    if (!rp) {
        return false;
    }

    const AbstractPlan* abstractPlan = rp->getActivePlan();
    if (abstractPlan && abstractPlan->getId() == id) {
        return true;
    }

    for (const auto& child : rp->getChildren()) {
        if (isPlanActiveHelper(child, id)) {
            return true;
        }
    }
    return false;
}

bool Util::hasPlanSucceeded(alica::AlicaEngine* ae, int64_t id)
{
    return hasPlanSucceededHelper(ae->getPlanBase().getRootNode(), id);
}

bool Util::hasPlanSucceededHelper(const RunningPlan* rp, int64_t id)
{
    if (!rp) {
        return false;
    }

    const AbstractPlan* abstractPlan = rp->getActivePlan();
    if (abstractPlan && abstractPlan->getId() == id) {
        return rp->amISuccessful();
    }

    for (const auto& child : rp->getChildren()) {
        if (hasPlanSucceededHelper(child, id)) {
            return true;
        }
    }
    return false;
}

const alica::Agent* Util::getLocalAgent(alica::AlicaEngine* ae)
{
    return ae->getTeamManager().getLocalAgent();
}

int Util::getTeamSize(alica::AlicaEngine* ae)
{
    return ae->getTeamManager().getTeamSize();
}

const alica::Agent* Util::getAgentByID(alica::AlicaEngine* ae, alica::AgentId agentID)
{
    return ae->getTeamManager().getAgentByID(agentID);
}
} // namespace alica::test