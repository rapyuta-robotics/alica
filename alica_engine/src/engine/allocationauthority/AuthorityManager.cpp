#include "engine/allocationauthority/AuthorityManager.h"

#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/debug_output.h>

#include <assert.h>

namespace alica
{
/**
 * Constructor
 */
AuthorityManager::AuthorityManager(AlicaEngine* engine)
        : _engine(engine)
        , _localAgentID(0)
{
}

AuthorityManager::~AuthorityManager() {}

/**
 * Initialises this engine module
 */
void AuthorityManager::init()
{
    _localAgentID = _engine->getTeamManager().getLocalAgentID();
}

/**
 * Closes this engine module
 */
void AuthorityManager::close() {}

/**
 * Message Handler
 * param name = aai A AllocationAthorityInfo
 */
void AuthorityManager::handleIncomingAuthorityMessage(const AllocationAuthorityInfo& aai)
{
    AlicaTime now = _engine->getAlicaClock().now();
    TeamManager& tm = _engine->editTeamManager();
    if (aai.senderID == _localAgentID || tm.isAgentIgnored(aai.senderID)) {
        return;
    }

    tm.setTimeLastMsgReceived(aai.senderID, now);
    if (_localAgentID < aai.senderID) {
        // notify TO that evidence about other robots is available
        for (EntryPointRobots epr : aai.entryPointRobots) {
            for (uint64_t rid : epr.robots) {
                if (rid != _localAgentID) {
                    tm.setTimeLastMsgReceived(rid, now);
                }
            }
        }
    }

    ALICA_DEBUG_MSG("AM: Received AAI Assignment: " << aai);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_back(aai);
    }
}

/**
 * Cyclic tick function, called by the plan base every iteration
 */
void AuthorityManager::tick(RunningPlan* rp)
{
    ALICA_DEBUG_MSG("AM: Tick called!");

    std::lock_guard<std::mutex> lock(_mutex);
    if (rp) {
        processPlan(*rp);
    }
    _queue.clear();
}

void AuthorityManager::processPlan(RunningPlan& rp)
{
    if (rp.isBehaviour()) {
        return;
    }
    if (rp.getCycleManagement().needsSending()) {
        sendAllocation(rp);
        rp.editCycleManagement().sent();
    }

    ALICA_DEBUG_MSG("AM: Queue size of AuthorityInfos is " << _queue.size());

    for (int i = 0; i < static_cast<int>(_queue.size()); ++i) {
        if (authorityMatchesPlan(_queue[i], rp)) {
            ALICA_DEBUG_MSG("AM: Found AuthorityInfo, which matches the plan " << rp.getActivePlan()->getName());
            rp.editCycleManagement().handleAuthorityInfo(_queue[i]);
            _queue.erase(_queue.begin() + i);
            --i;
        }
    }
    for (RunningPlan* c : rp.getChildren()) {
        processPlan(*c);
    }
}
/**
 * Sends an AllocationAuthorityInfo message containing the assignment of p
 */
void AuthorityManager::sendAllocation(const RunningPlan& p)
{
    if (!_engine->maySendMessages()) {
        return;
    }
    AllocationAuthorityInfo aai{};

    const Assignment& ass = p.getAssignment();
    for (int i = 0; i < ass.getEntryPointCount(); ++i) {
        EntryPointRobots epRobots;
        epRobots.entrypoint = ass.getEntryPoint(i)->getId();
        ass.getAgentsWorking(i, epRobots.robots);

        aai.entryPointRobots.push_back(std::move(epRobots));
    }

    const RunningPlan* parent = p.getParent();
    aai.parentState = ((parent && parent->getActiveState()) ? parent->getActiveState()->getId() : -1);
    aai.planId = p.getActivePlan()->getId();
    aai.authority = _localAgentID;
    aai.senderID = _localAgentID;
    aai.planType = (p.getPlanType() ? p.getPlanType()->getId() : -1);

    ALICA_DEBUG_MSG("AM: Sending AAI Assignment: " << aai);
    _engine->getCommunicator().sendAllocationAuthority(aai);
}

/**
 Matches a plan based on the context: a plan, together with the containing plantype and state is matched.
 */
bool AuthorityManager::authorityMatchesPlan(const AllocationAuthorityInfo& aai, const RunningPlan& p) const
{
    assert(!p.isRetired());
    // If a plan is not retired and does not have a parent, it must be masterplan
    if (p.isRetired()) {
        return false;
    }
    const RunningPlan* parent = p.getParent();
    if ((parent == nullptr && aai.parentState == -1) ||
            (parent != nullptr && parent->getActiveState() != nullptr && parent->getActiveState()->getId() == aai.parentState)) {
        if (p.getActivePlan()->getId() == aai.planId) {
            return true;
        } else if (aai.planType != -1 && p.getPlanType() != nullptr && p.getPlanType()->getId() == aai.planType) {
            return true;
        }
    }
    return false;
}

} // namespace alica
