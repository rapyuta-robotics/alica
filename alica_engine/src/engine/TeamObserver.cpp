#include "engine/TeamObserver.h"
#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/SimplePlanTree.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
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
#include "supplementary/AgentID.h"
#include <SystemConfig.h>

#include <alica_common_config/debug_output.h>

namespace alica
{

using std::cout;
using std::endl;
using std::list;
using std::lock_guard;
using std::make_shared;
using std::map;
using std::mutex;
using std::pair;
using std::to_string;

TeamObserver::TeamObserver(AlicaEngine* ae)
        : ae(ae)
        , teamManager(ae->getTeamManager())
{
    this->simplePlanTrees = make_shared<map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>(
            map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>());
    this->myId = this->teamManager->getLocalAgentID();
    this->me = this->teamManager->getAgentByID(this->myId)->getEngineData();
}

TeamObserver::~TeamObserver() {}

std::unique_ptr<map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>> TeamObserver::getTeamPlanTrees()
{
    auto ret = std::unique_ptr<map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>(
            new map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>);
    lock_guard<mutex> lock(this->simplePlanTreeMutex);
    // TODO get rid of this once teamManager gets a datastructure overhaul
    AgentGrp tmp;
    teamManager->fillWithActiveAgentIDs(tmp);
    for (const supplementary::AgentID* agentId : tmp) {
        auto iter = this->simplePlanTrees->find(agentId);
        if (iter != simplePlanTrees->end() && iter->second != nullptr) {
            ret->insert(pair<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>>(agentId, iter->second));
        }
    }
    return ret;
}

void TeamObserver::tick(RunningPlan* root)
{
    AlicaTime time = ae->getAlicaClock()->now();

    bool changedSomeAgent = false;
    bool changedCurrentAgent = false;
    std::unique_ptr<std::list<Agent*>> agents = this->teamManager->getAllAgents();

    for (auto agent : *agents) {
        changedCurrentAgent = agent->update();
        if (changedCurrentAgent && !agent->isActive()) {
            lock_guard<mutex> lock(this->simplePlanTreeMutex);
            this->simplePlanTrees->erase(agent->getID());
        }
        changedSomeAgent |= changedCurrentAgent;
    }

    // notifications for teamchanges, you can add some code below if you want to be notified when the team changed
    if (changedSomeAgent) {
        ae->getRoleAssignment()->update();
        this->ae->getLog()->eventOccurred("TeamChanged");
    }

    cleanOwnSuccessMarks(root);
    if (root != nullptr) {
        // TODO get rid of this once teamManager gets a datastructure overhaul
        AgentGrp activeAgents;
        teamManager->fillWithActiveAgentIDs(activeAgents);

        list<std::shared_ptr<SimplePlanTree>> updatespts;
        list<const supplementary::AgentID*> noUpdates;
        lock_guard<mutex> lock(this->simplePlanTreeMutex);
        for (auto iterator = this->simplePlanTrees->begin(); iterator != this->simplePlanTrees->end(); iterator++) {
            if (!this->teamManager->isAgentActive(iterator->second->getRobotId())) {
                continue;
            }

            if (iterator->second->isNewSimplePlanTree()) {
                updatespts.push_back(iterator->second);
                ALICA_DEBUG_MSG("TO: added to update");
                iterator->second->setNewSimplePlanTree(false);
            } else {
                ALICA_DEBUG_MSG("TO: added to noupdate");
                noUpdates.push_back(iterator->second->getRobotId());
            }
        }
        ALICA_DEBUG_MSG("TO: spts size " << updatespts.size());

        if (root->recursiveUpdateAssignment(updatespts, activeAgents, noUpdates, time)) {
            this->ae->getLog()->eventOccurred("MsgUpdate");
        }
    }
}

void TeamObserver::close()
{
    std::cout << "TO: Closed Team Observer" << std::endl;
}

/**
 * Broadcasts a PlanTreeInfo Message
 * @param msg A list of long, a serialized version of the current planning tree
 * as constructed by RunningPlan.ToMessage.
 */
void TeamObserver::doBroadCast(const IdGrp& msg) const
{
    if (!ae->maySendMessages()) {
        return;
    }
    PlanTreeInfo pti = PlanTreeInfo();
    pti.senderID = this->myId;
    pti.stateIDs = msg;
    pti.succeededEPs = this->me->getSuccessMarks()->toIdGrp();
    ae->getCommunicator()->sendPlanTreeInfo(pti);
#ifdef TO_DEBUG
    cout << "TO: Sending Plan Message: " << endl;
    for (int i = 0; i < msg.size(); i++) {
        list<long>::const_iterator iter = msg.begin();
        advance(iter, i);
        cout << *iter << "\t";
    }
    cout << endl;
#endif
}

/**
 * Removes any successmarks left by this robot in plans no longer inhabited by any agent.
 * @param root a shared_ptr of a RunningPlan
 */
void TeamObserver::cleanOwnSuccessMarks(RunningPlan* root)
{
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
    list<std::shared_ptr<SimplePlanTree>> queue;
    lock_guard<mutex> lock(this->simplePlanTreeMutex);
    for (auto pair : *this->simplePlanTrees) {
        if (pair.second.operator bool()) {
            queue.push_back(pair.second);
        }
    }
    while (queue.size() > 0) {
        std::shared_ptr<SimplePlanTree> spt = queue.front();
        queue.pop_front();
        presentPlans.push_back(spt->getState()->getInPlan());
        for (const std::shared_ptr<SimplePlanTree>& c : spt->getChildren()) {
            queue.push_back(c);
        }
    }
    this->me->getSuccessMarks()->limitToPlans(presentPlans);
}

const EntryPoint* TeamObserver::entryPointOfState(const State* state) const
{
    for (const EntryPoint* ep : state->getInPlan()->getEntryPoints()) {
        if (std::find(ep->getReachableStates().begin(), ep->getReachableStates().end(), state) != ep->getReachableStates().end()) {
            return ep;
        }
    }
    return nullptr;
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
    auto tmp = this->teamManager->getActiveAgents();
    for (auto& agent : *tmp) {
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(plan);
        }
        if (suc != nullptr) {
            ret += suc->size();
        }
    }
    suc = me->getSuccessMarks()->succeededEntryPoints(plan);
    if (suc != nullptr) {
        ret += suc->size();
    }
    return ret;
}

SuccessCollection TeamObserver::createSuccessCollection(const Plan* plan) const
{
    SuccessCollection ret(plan);

    auto tmp = this->teamManager->getActiveAgents();
    for (const Agent* agent : *tmp) {
        const EntryPointGrp* suc = nullptr;
        if (teamManager->getLocalAgent() == agent) {
            continue;
        }
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(plan);
        }
        if (suc != nullptr) {
            for (const EntryPoint* ep : *suc) {
                ret.setSuccess(agent->getID(), ep);
            }
        }
    }
    const EntryPointGrp* suc = me->getSuccessMarks()->succeededEntryPoints(plan);
    if (suc != nullptr) {
        for (const EntryPoint* ep : *suc) {
            ret.setSuccess(myId, ep);
        }
    }
    return ret;
}

void TeamObserver::updateSuccessCollection(const Plan* p, std::shared_ptr<SuccessCollection> sc)
{
    sc->clear();
    const EntryPointGrp* suc = nullptr;
    auto tmp = this->teamManager->getActiveAgents();
    for (auto& agent : *tmp) {
        {
            lock_guard<mutex> lock(this->successMarkMutex);
            suc = agent->getSucceededEntryPoints(p);
        }
        if (suc != nullptr) {
            for (const EntryPoint* ep : *suc) {
                sc->setSuccess(agent->getID(), ep);
            }
        }
    }
    suc = me->getSuccessMarks()->succeededEntryPoints(p);
    if (suc != nullptr) {
        for (const EntryPoint* ep : *suc) {
            sc->setSuccess(myId, ep);
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
    lock_guard<mutex> lock(this->simplePlanTreeMutex);
    for (auto iterator : *this->simplePlanTrees) {
        if (iterator.second->containsPlan(plan)) {
            return;
        }
    }
    this->me->getSuccessMarks()->removePlan(plan);
}

void TeamObserver::handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming)
{
    if (*(incoming->senderID) != *myId) {
        if (this->teamManager->isAgentIgnored(incoming->senderID)) {
            return;
        }
        auto spt = sptFromMessage(incoming->senderID, incoming->stateIDs);
        if (spt != nullptr) {
            this->teamManager->setTimeLastMsgReceived(incoming->senderID, ae->getAlicaClock()->now());
            {
                lock_guard<mutex> lock(this->successMarkMutex);
                this->teamManager->setSuccessMarks(incoming->senderID, std::make_shared<SuccessMarks>(ae, incoming->succeededEPs));
            }

            lock_guard<mutex> lock(this->simplePlanTreeMutex);
            auto sptEntry = this->simplePlanTrees->find(incoming->senderID);
            if (sptEntry != this->simplePlanTrees->end()) {
                sptEntry->second = spt;
            } else {
                this->simplePlanTrees->insert(pair<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>>(incoming->senderID, spt));
            }
        }
    }
}

/**
 * Constructs a SimplePlanTree from a received message
 * @param robotId The id of the other robot.
 * @param ids The list of long encoding another robot's plantree as received in a PlanTreeInfo message.
 * @return shared_ptr of a SimplePlanTree
 */
std::shared_ptr<SimplePlanTree> TeamObserver::sptFromMessage(const supplementary::AgentID* robotId, const IdGrp& ids) const
{
#ifdef TO_DEBUG
    std::cout << "Spt from robot " << robotId << std::endl;
    ;
    for (int64_t id : ids) {
        std::cout << id << "\t";
    }
    std::cout << std::endl;
#endif
    if (ids.size() == 0) {
        // warning
        std::cerr << "TO: Empty state list for robot " << robotId << std::endl;
        return nullptr;
    }

    AlicaTime time = ae->getAlicaClock()->now();
    std::shared_ptr<SimplePlanTree> root = make_shared<SimplePlanTree>();
    root->setRobotId(robotId);
    root->setReceiveTime(time);
    root->setStateIds(ids);

    auto iter = ids.begin();
    const PlanRepository::Accessor<State>& states = ae->getPlanRepository()->getStates();
    const State* s = states.find(*iter);
    if (s) {
        root->setState(s);
        root->setEntryPoint(entryPointOfState(s));
        if (root->getEntryPoint() == nullptr) {
            // Warning
            IdGrp::const_iterator iter = ids.begin();
            cout << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
            return nullptr;
        }
    } else {
        return nullptr;
    }

    std::shared_ptr<SimplePlanTree> curParent;
    std::shared_ptr<SimplePlanTree> cur = root;
    if (ids.size() > 1) {
        IdGrp::const_iterator iter = ids.begin();
        iter++;
        for (; iter != ids.end(); iter++) {
            if (*iter == -1) {
                curParent = cur;
                cur = nullptr;
            } else if (*iter == -2) {
                cur = curParent;
                if (cur == nullptr) {
                    cout << "TO: Malformed SptMessage from " << robotId << endl;
                    return nullptr;
                }
            } else {
                cur = make_shared<SimplePlanTree>();
                cur->setRobotId(robotId);
                cur->setReceiveTime(time);

                curParent->editChildren().insert(cur);
                const State* s2 = states.find(*iter);
                if (s2) {
                    cur->setState(s2);
                    cur->setEntryPoint(entryPointOfState(s2));
                    if (cur->getEntryPoint() == nullptr) {
                        IdGrp::const_iterator iter = ids.begin();
                        cout << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
                        return nullptr;
                    }
                } else {
                    IdGrp::const_iterator iter = ids.begin();
                    cout << "Unknown State (" << *iter << ") received from " << robotId << endl;
                    return nullptr;
                }
            }
        }
    }

    return root;
}

} /* namespace alica */
