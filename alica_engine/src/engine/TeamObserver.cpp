#include "engine/TeamObserver.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/AlicaClock.h"
#include "engine/IAlicaCommunication.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/SimplePlanTree.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/Logger.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Characteristic.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/Agent.h"
#include "supplementary/AgentID.h"
#include "engine/AlicaClock.h"
#include "engine/IRoleAssignment.h"
#include <SystemConfig.h>

namespace alica {

TeamObserver::TeamObserver(AlicaEngine* ae)
        : ae(ae)
        , teamManager(ae->getTeamManager()) {
    this->simplePlanTrees = make_shared<
            map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>(
            map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>());
    this->myId = this->teamManager->getLocalAgentID();
    this->me = this->teamManager->getAgentByID(this->myId)->getEngineData();
}

TeamObserver::~TeamObserver() {}

unique_ptr<map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>
TeamObserver::getTeamPlanTrees() {
    auto ret = unique_ptr<
            map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>(
            new map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>);
    lock_guard<mutex> lock(this->simplePlanTreeMutex);

    std::vector<const supplementary::AgentID*> tmp; //TODO get rid of this once teamManager gets a datastructure overhaul
    teamManager->fillWithActiveAgentIDs(tmp);
    for (const supplementary::AgentID* agentId : tmp) {
        auto iter = this->simplePlanTrees->find(agentId);
        if (iter != simplePlanTrees->end() && iter->second != nullptr) {
            ret->insert(pair<const supplementary::AgentID*, shared_ptr<SimplePlanTree>>(agentId, iter->second));
        }
    }
    return move(ret);
}

void TeamObserver::tick(shared_ptr<RunningPlan> root) {
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
        this->ae->getLog()->eventOccured("TeamChanged");
    }

    cleanOwnSuccessMarks(root);
    if (root != nullptr) {
        std::vector<const supplementary::AgentID*> activeAgents; //TODO get rid of this once teamManager gets a datastructure overhaul
        teamManager->fillWithActiveAgentIDs(activeAgents);

        list<shared_ptr<SimplePlanTree>> updatespts;
        list<const supplementary::AgentID*> noUpdates;
        lock_guard<mutex> lock(this->simplePlanTreeMutex);
        for (auto iterator = this->simplePlanTrees->begin(); iterator != this->simplePlanTrees->end(); iterator++) {
            if (!this->teamManager->isAgentActive(iterator->second->getRobotId())) {
                continue;
            }

            if (iterator->second->isNewSimplePlanTree()) {
                updatespts.push_back(iterator->second);
#ifdef TO_DEBUG
                cout << "TO: added to update" << endl;
#endif
                iterator->second->setNewSimplePlanTree(false);
            } else {
#ifdef TO_DEBUG
                cout << "TO: added to noupdate" << endl;
#endif
                noUpdates.push_back(iterator->second->getRobotId());
            }
        }
#ifdef TO_DEBUG
        cout << "TO: spts size " << updatespts.size() << endl;
#endif

        if (root->recursiveUpdateAssignment(updatespts, activeAgents, noUpdates, time)) {
            this->ae->getLog()->eventOccured("MsgUpdate");
        }
    }
}

void TeamObserver::close() {
    cout << "TO: Closed Team Observer" << endl;
}

/**
 * Broadcasts a PlanTreeInfo Message
 * @param msg A list of long, a serialized version of the current planning tree
 * as constructed by RunningPlan.ToMessage.
 */
void TeamObserver::doBroadCast(list<long>& msg) {
    if (!ae->maySendMessages) {
        return;
    }
    PlanTreeInfo pti = PlanTreeInfo();
    pti.senderID = this->myId;
    pti.stateIDs = msg;
    pti.succeededEPs = this->me->getSuccessMarks()->toList();
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
void TeamObserver::cleanOwnSuccessMarks(shared_ptr<RunningPlan> root) {
    unique_ptr<unordered_set<AbstractPlan*>> presentPlans =
            unique_ptr<unordered_set<AbstractPlan*>>(new unordered_set<AbstractPlan*>);
    if (root != nullptr) {
        list<shared_ptr<RunningPlan>>* q = new list<shared_ptr<RunningPlan>>();
        q->push_front(root);
        while (q->size() > 0) {
            shared_ptr<RunningPlan> p = q->front();
            q->pop_front();
            if (!p->isBehaviour()) {
                presentPlans->insert(p->getPlan());
                for (shared_ptr<RunningPlan> c : *p->getChildren()) {
                    q->push_back(c);
                }
            }
        }
        delete q;
    }
    list<shared_ptr<SimplePlanTree>> queue;
    lock_guard<mutex> lock(this->simplePlanTreeMutex);
    for (auto pair : *this->simplePlanTrees) {
        if (pair.second.operator bool()) {
            queue.push_back(pair.second);
        }
    }
    while (queue.size() > 0) {
        shared_ptr<SimplePlanTree> spt = queue.front();
        queue.pop_front();
        presentPlans->insert(spt->getState()->getInPlan());
        for (shared_ptr<SimplePlanTree> c : spt->getChildren()) {
            queue.push_back(c);
        }
    }
    this->me->getSuccessMarks()->limitToPlans(move(presentPlans));
}

EntryPoint* TeamObserver::entryPointOfState(State* state) {
    for (auto iter = state->getInPlan()->getEntryPoints().begin(); iter != state->getInPlan()->getEntryPoints().end();
            iter++) {
        if (iter->second->getReachableStates().find(state) != iter->second->getReachableStates().end()) {
            return iter->second;
        }
    }
    return nullptr;
}

/**
 * Returns the number of successes the team knows about in the given plan.
 * @param plan a plan
 * @return an int counting successes in plan
 */
int TeamObserver::successesInPlan(Plan* plan) {
    int ret = 0;
    auto suc = make_shared<list<EntryPoint*>>(list<EntryPoint*>());
    auto tmp = this->teamManager->getActiveAgents();
    for (auto& agent : *tmp) {
        {
            lock_guard<mutex> lock(this->successMark);
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

shared_ptr<SuccessCollection> TeamObserver::getSuccessCollection(Plan* plan) {
    shared_ptr<SuccessCollection> ret = make_shared<SuccessCollection>(plan);
    shared_ptr<list<EntryPoint*>> suc;
    auto tmp = this->teamManager->getActiveAgents();
    for (const Agent* agent : *tmp) {
        if (teamManager->getLocalAgent() == agent) {
            continue;
        }
        {
            lock_guard<mutex> lock(this->successMark);
            suc = agent->getSucceededEntryPoints(plan);
        }
        if (suc != nullptr) {
            for (EntryPoint* ep : *suc) {
                ret->setSuccess(agent->getID(), ep);
            }
        }
    }
    suc = me->getSuccessMarks()->succeededEntryPoints(plan);
    if (suc != nullptr) {
        for (EntryPoint* ep : *suc) {
            ret->setSuccess(myId, ep);
        }
    }
    return ret;
}

void TeamObserver::updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc) {
    sc->clear();
    shared_ptr<list<EntryPoint*>> suc;
    auto tmp = this->teamManager->getActiveAgents();
    for (auto& agent : *tmp) {
        {
            lock_guard<mutex> lock(this->successMark);
            suc = agent->getSucceededEntryPoints(p);
        }
        if (suc != nullptr) {
            for (EntryPoint* ep : *suc) {
                sc->setSuccess(agent->getID(), ep);
            }
        }
    }
    suc = me->getSuccessMarks()->succeededEntryPoints(p);
    if (suc != nullptr) {
        for (EntryPoint* ep : *suc) {
            sc->setSuccess(myId, ep);
        }
    }
}

/**
 * Notify the TeamObserver that this robot has left a plan. This will reset any successmarks left, if no other robot is
 * believed to be left in the plan.
 * @param plan The AbstractPlan left by the robot
 */
void TeamObserver::notifyRobotLeftPlan(AbstractPlan* plan) {
    lock_guard<mutex> lock(this->simplePlanTreeMutex);
    for (auto iterator : *this->simplePlanTrees) {
        if (iterator.second->containsPlan(plan)) {
            return;
        }
    }
    this->me->getSuccessMarks()->removePlan(plan);
}

void TeamObserver::handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming) {
    if (*(incoming->senderID) != *myId) {
        if (this->teamManager->isAgentIgnored(incoming->senderID)) {
            return;
        }
        auto spt = sptFromMessage(incoming->senderID, incoming->stateIDs);
        if (spt != nullptr) {
            this->teamManager->setTimeLastMsgReceived(incoming->senderID, ae->getAlicaClock()->now());
            {
                lock_guard<mutex> lock(this->successMark);
                this->teamManager->setSuccessMarks(
                        incoming->senderID, make_shared<SuccessMarks>(ae, incoming->succeededEPs));
            }

            lock_guard<mutex> lock(this->simplePlanTreeMutex);
            auto sptEntry = this->simplePlanTrees->find(incoming->senderID);
            if (sptEntry != this->simplePlanTrees->end()) {
                sptEntry->second = spt;
            } else {
                this->simplePlanTrees->insert(
                        pair<const supplementary::AgentID*, shared_ptr<SimplePlanTree>>(incoming->senderID, spt));
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
shared_ptr<SimplePlanTree> TeamObserver::sptFromMessage(const supplementary::AgentID* robotId, list<long> ids) {
#ifdef TO_DEBUG
    cout << "Spt from robot " << robotId << endl;
    ;
    for (int i = 0; i < ids.size(); i++) {
        list<long>::const_iterator iter = ids.begin();
        advance(iter, i);
        cout << *iter << "\t";
    }
    cout << endl;
#endif
    if (ids.size() == 0) {
        // warning
        cerr << "TO: Empty state list for robot " << robotId << endl;
        return nullptr;
    }
    map<long, State*>& states = ae->getPlanRepository()->getStates();
    AlicaTime time = ae->getAlicaClock()->now();
    shared_ptr<SimplePlanTree> root = make_shared<SimplePlanTree>();
    root->setRobotId(robotId);
    root->setReceiveTime(time);
    root->setStateIds(ids);
    State* s = nullptr;
    auto iter = ids.begin();
    if (states.find(*iter) != states.end()) {
        root->setState(states.at(*iter));
        root->setEntryPoint(entryPointOfState(root->getState()));
        if (root->getEntryPoint() == nullptr) {
            // Warning
            list<long>::const_iterator iter = ids.begin();
            cout << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
            return nullptr;
        }
    } else {
        list<long>::const_iterator iter = ids.begin();
        // cout << "TO: Unknown State (" << *iter << ") received from " << robotId << endl;
        return nullptr;
    }

    shared_ptr<SimplePlanTree> curParent;
    shared_ptr<SimplePlanTree> cur = root;
    if (ids.size() > 1) {
        list<long>::const_iterator iter = ids.begin();
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

                curParent->getChildren().insert(cur);
                if (states.find(*iter) != states.end()) {
                    cur->setState(states.at(*iter));
                    cur->setEntryPoint(entryPointOfState(cur->getState()));
                    if (cur->getEntryPoint() == nullptr) {
                        list<long>::const_iterator iter = ids.begin();
                        cout << "TO: Cannot find ep for State (" << *iter << ") received from " << robotId << endl;
                        return nullptr;
                    }
                } else {
                    list<long>::const_iterator iter = ids.begin();
                    cout << "Unknown State (" << *iter << ") received from " << robotId << endl;
                    return nullptr;
                }
            }
        }
    }

    return root;
}

} /* namespace alica */
