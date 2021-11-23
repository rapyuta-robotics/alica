#include "alica/test/Util.h"

#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>
#include <engine/Types.h>

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
    const std::vector<RunningPlan*>& children = rp->getChildren();
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        if (isStateActiveHelper(children[i], id)) {
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
    const std::vector<RunningPlan*>& children = rp->getChildren();
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        if (isPlanActiveHelper(children[i], id)) {
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