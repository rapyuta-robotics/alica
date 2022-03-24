#include "alica/test/Util.h"

#include <engine/Types.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>

namespace alica::test
{
BasicBehaviour* Util::getBasicBehaviour(alica::AlicaEngine* ae, int64_t behaviourID, int64_t configurationID)
{
    return getBasicBehaviourHelper(ae->getPlanBase().getRootNode(), behaviourID);
}

BasicBehaviour* Util::getBasicBehaviourHelper(const RunningPlan* rp, int64_t behaviourId)
{
    if (!rp) {
        return nullptr;
    }

    if (rp->isBehaviour() && rp->getBasicBehaviour()->getId() == behaviourId) {
        return rp->getBasicBehaviour();
    }

    for (const auto& child : rp->getChildren()) {
        BasicBehaviour* beh = getBasicBehaviourHelper(child, behaviourId);
        if (beh) {
            return beh;
        }
    }

    return nullptr;
}

BasicPlan* Util::getBasicPlan(alica::AlicaEngine* ae, int64_t planId, int64_t configurationId)
{
    BasicPlan* plan = nullptr;
    for (auto& planEntry : ae->getPlanPool().getAvailablePlans()) {
        if (planEntry.first.first->getId() == planId &&
                (configurationId == 0 ? planEntry.first.second == nullptr : planEntry.first.second->getId() == configurationId)) {
            plan = planEntry.second.get();
            break;
        }
    }
    return plan;
    // return getBasicPlanHelper(ae->getPlanBase().getRootNode(), planId);
}

BasicPlan* Util::getBasicPlanHelper(const RunningPlan* rp, int64_t planId)
{
    if (!rp) {
        return nullptr;
    }

    BasicPlan* plan = rp->getBasicPlan();
    if (plan /*&& plan->getId() == planId*/) {
        return plan;
    }

    for (const auto& child : rp->getChildren()) {
        plan = getBasicPlanHelper(child, planId);
        if (plan) {
            return plan;
        }
    }

    return nullptr;
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
