#include "engine/TeamObserver.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/SimplePlanTree.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Characteristic.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/Agent.h"
#include "engine/teammanager/TeamManager.h"
#include <SystemConfig.h>

#include <engine/AgentIDConstPtr.h>
#include <engine/Output.h>

namespace alica
{

using std::lock_guard;
using std::map;
using std::mutex;
using std::pair;

TeamObserver::TeamObserver(AlicaEngine* ae)
        : _ae(ae)
        , _tm(ae->getTeamManager())
        , _myId(ae->getTeamManager()->getLocalAgentID())
{
    _me = _tm->getAgentByID(_myId);
}

TeamObserver::~TeamObserver() {}

bool TeamObserver::updateTeamPlanTrees()
{
    { // Get stuff out of the queue
        lock_guard<mutex> queueLock(_msgQueueMutex);

        for (const auto& msg : _msgQueue) {
            std::unique_ptr<SimplePlanTree> spt = sptFromMessage(msg.first->senderID, msg.first->stateIDs, msg.second);
            if (spt != nullptr) {
                _tm->setTimeLastMsgReceived(msg.first->senderID, msg.second);
                _tm->setSuccessMarks(msg.first->senderID, std::make_shared<SuccessMarks>(_ae, msg.first->succeededEPs));

                auto sptEntry = _simplePlanTrees.find(spt->getAgentId());
                if (sptEntry != _simplePlanTrees.end()) {
                    sptEntry->second = std::move(spt);
                } else {
                    _simplePlanTrees.emplace(spt->getAgentId(), std::move(spt));
                }
            }
        }
        _msgQueue.clear();
    }

    bool changedSomeAgent = false;
    for (const auto& agent : _tm->getAllAgents()) {
        bool changedCurrentAgent = agent.second->update();
        if (changedCurrentAgent && !agent.second->isActive()) {
            _simplePlanTrees.erase(agent.second->getId());
        }
        changedSomeAgent |= changedCurrentAgent;
    }
    return changedSomeAgent;
}

void TeamObserver::tick(RunningPlan* root)
{
    AlicaTime time = _ae->getAlicaClock()->now();

    bool someChanges = updateTeamPlanTrees();
    // notifications for teamchanges, you can add some code below if you want to be notified when the team changed
    if (someChanges) {
        _ae->getRoleAssignment()->update();
        _ae->getLog()->eventOccurred("TeamChanged");
    }

    cleanOwnSuccessMarks(root);
    if (root != nullptr) {
        // TODO get rid of this once teamManager gets a datastructure overhaul
        AgentGrp activeAgents;
        std::copy(_tm->getActiveAgentIds().begin(), _tm->getActiveAgentIds().end(), std::back_inserter(activeAgents));

        std::vector<const SimplePlanTree*> updatespts;
        AgentGrp noUpdates;
        for (auto& ele : _simplePlanTrees) {
            assert(_tm->isAgentActive(ele.second->getAgentId()));

            if (ele.second->isNewSimplePlanTree()) {
                updatespts.push_back(ele.second.get());
                ALICA_DEBUG_MSG("TO: added to update");
                ele.second->setProcessed();
            } else {
                ALICA_DEBUG_MSG("TO: added to noupdate");
                noUpdates.push_back(ele.second->getAgentId());
            }
        }
        ALICA_DEBUG_MSG("TO: spts size " << updatespts.size());

        if (root->recursiveUpdateAssignment(updatespts, activeAgents, noUpdates, time)) {
            _ae->getLog()->eventOccurred("MsgUpdate");
        }
    }
}

void TeamObserver::close()
{
    ALICA_INFO_MSG("TO: Closed Team Observer");
}

/**
 * Broadcasts a PlanTreeInfo Message
 * @param msg A list of long, a serialized version of the current planning tree
 * as constructed by RunningPlan.ToMessage.
 */
void TeamObserver::doBroadCast(const IdGrp& msg) const
{
    if (!_ae->maySendMessages()) {
        return;
    }
    PlanTreeInfo pti = PlanTreeInfo();
    pti.senderID = _myId;
    pti.stateIDs = msg;
    pti.succeededEPs = _me->getEngineData().getSuccessMarks()->toIdGrp();
    _ae->getCommunicator()->sendPlanTreeInfo(pti);

    ALICA_DEBUG_MSG("TO: Sending Plan Message: " << msg);
}

/**
 * Removes any successmarks left by this robot in plans no longer inhabited by any agent.
 * @param root a shared_ptr of a RunningPlan
 */
void TeamObserver::cleanOwnSuccessMarks(RunningPlan* root)
{
    // TODO: clean this up
    AbstractPlanGrp presentPlans;
    if (root != nullptr) {
        std::list<RunningPlan*> q;
        q.push_front(root);
        while (q.size() > 0) {
            RunningPlan* p = q.front();
            q.pop_front();
            if (!p->isBehaviour()) {
                presentPlans.push_back(p->getActivePlan());
                for (RunningPlan* c : p->getChildren()) {
                    q.push_back(c);
                }
            }
        }
    }
    std::vector<const SimplePlanTree*> queue;
    for (const auto& pair : _simplePlanTrees) {
        if (pair.second.operator bool()) {
            queue.push_back(pair.second.get());
        }
    }
    while (queue.size() > 0) {
        const SimplePlanTree* spt = queue.back();
        queue.pop_back();
        presentPlans.push_back(spt->getState()->getInPlan());
        for (const std::unique_ptr<SimplePlanTree>& c : spt->getChildren()) {
            queue.push_back(c.get());
        }
    }
    _me->getEngineData().getSuccessMarks()->limitToPlans(presentPlans);
}

/**
 * Returns the number of successes the team knows about in the given plan.
 * @param plan a plan
 * @return an int counting successes in plan
 */
int TeamObserver::successesInPlan(const Plan* plan)
{
    int ret = 0;
    const EntryPointGrp* suc = nullptr;
    for (const Agent* agent : _tm->getActiveAgents()) {
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(plan);
        }
        if (suc != nullptr) {
            ret += suc->size();
        }
    }
    suc = _me->getEngineData().getSuccessMarks()->succeededEntryPoints(plan);
    if (suc != nullptr) {
        ret += suc->size();
    }
    return ret;
}

SuccessCollection TeamObserver::createSuccessCollection(const Plan* plan) const
{
    SuccessCollection ret(plan);

    for (const Agent* agent : _tm->getActiveAgents()) {
        const EntryPointGrp* suc = nullptr;
        if (_myId == agent->getId()) {
            continue;
        }
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(plan);
        }
        if (suc != nullptr) {
            for (const EntryPoint* ep : *suc) {
                ret.setSuccess(agent->getId(), ep);
            }
        }
    }
    const EntryPointGrp* suc = _me->getEngineData().getSuccessMarks()->succeededEntryPoints(plan);
    if (suc != nullptr) {
        for (const EntryPoint* ep : *suc) {
            ret.setSuccess(_myId, ep);
        }
    }
    return ret;
}

void TeamObserver::updateSuccessCollection(const Plan* p, SuccessCollection& sc)
{
    sc.clear();
    const EntryPointGrp* suc = nullptr;

    for (const Agent* agent : _tm->getActiveAgents()) {
        if (agent->getId() == _myId) {
            continue;
        }
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(p);
        }
        if (suc != nullptr) {
            for (const EntryPoint* ep : *suc) {
                sc.setSuccess(agent->getId(), ep);
            }
        }
    }
    suc = _me->getEngineData().getSuccessMarks()->succeededEntryPoints(p);
    if (suc != nullptr) {
        for (const EntryPoint* ep : *suc) {
            sc.setSuccess(_myId, ep);
        }
    }
}

/**
 * Notify the TeamObserver that this robot has left a plan. This will reset any successmarks left, if no other robot is
 * believed to be left in the plan.
 * @param plan The AbstractPlan left by the robot
 */
void TeamObserver::notifyRobotLeftPlan(const AbstractPlan* plan)
{
    for (const auto& ele : _simplePlanTrees) {
        if (ele.second->containsPlan(plan)) {
            return;
        }
    }
    _me->getEngineData().getSuccessMarks()->removePlan(plan);
}

void TeamObserver::handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming)
{
    if (incoming->senderID != _myId) {
        if (_tm->isAgentIgnored(incoming->senderID)) {
            return;
        }
        lock_guard<mutex> lock(_msgQueueMutex);
        _msgQueue.emplace_back(std::move(incoming), _ae->getAlicaClock()->now());
    }
}

/**
 * Constructs a SimplePlanTree from a received message
 * @param robotId The id of the other robot.
 * @param ids The list of long encoding another robot's plantree as received in a PlanTreeInfo message.
 * @return shared_ptr of a SimplePlanTree
 */
std::unique_ptr<SimplePlanTree> TeamObserver::sptFromMessage(AgentIDConstPtr agentId, const IdGrp& ids, AlicaTime time) const
{
    ALICA_DEBUG_MSG("Spt from robot " << agentId);
    ALICA_DEBUG_MSG(ids);

    if (ids.empty()) {
        ALICA_ERROR_MSG("TO: Empty state list for agent " << agentId);
        return nullptr;
    }

    std::unique_ptr<SimplePlanTree> root{new SimplePlanTree()};
    root->setAgentId(agentId);
    root->setReceiveTime(time);
    root->setStateIds(ids);

    int64_t root_id = ids[0];
    const PlanRepository::Accessor<State>& states = _ae->getPlanRepository()->getStates();
    const State* s = states.find(root_id);
    if (s) {
        root->setState(s);
        root->setEntryPoint(s->getEntryPoint());
    } else {
        return nullptr;
    }

    SimplePlanTree* curParent = nullptr;
    SimplePlanTree* cur = root.get();

    for (int i = 1; i < static_cast<int>(ids.size()); ++i) {
        const int64_t id = ids[i];
        if (id == -1) {
            curParent = cur;
            cur = nullptr;
        } else if (id == -2) {
            cur = curParent;
            if (cur == nullptr) {
                ALICA_WARNING_MSG("TO: Malformed SptMessage from " << agentId);
                return nullptr;
            }
            curParent = cur->getParent();
        } else {
            cur = new SimplePlanTree();
            cur->setAgentId(agentId);
            cur->setReceiveTime(time);
            cur->setParent(curParent);
            curParent->editChildren().emplace_back(cur);
            const State* s2 = states.find(id);
            if (s2) {
                cur->setState(s2);
                cur->setEntryPoint(s2->getEntryPoint());
            } else {
                ALICA_WARNING_MSG("Unknown State (" << id << ") received from " << agentId);
                return nullptr;
            }
        }
    }
    return root;
}

} /* namespace alica */
