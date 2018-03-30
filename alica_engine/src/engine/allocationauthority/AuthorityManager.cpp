#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/model/State.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/PlanType.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/teammanager/TeamManager.h"

namespace alica {
/**
 * Constructor
 */
AuthorityManager::AuthorityManager(AlicaEngine* engine) : engine(engine), localAgentID(nullptr) {}

AuthorityManager::~AuthorityManager() {}

/**
 * Initialises this engine module
 */
void AuthorityManager::init() {
    this->localAgentID = engine->getTeamManager()->getLocalAgentID();
}

/**
 * Closes this engine module
 */
void AuthorityManager::close() {}

/**
 * Message Handler
 * param name = aai A AllocationAthorityInfo
 */
void AuthorityManager::handleIncomingAuthorityMessage(shared_ptr<AllocationAuthorityInfo> aai) {
    auto now = this->engine->getIAlicaClock()->now();
    if (this->engine->getTeamManager()->isAgentIgnored(aai->senderID)) {
        return;
    }
    if (*(aai->senderID) != *(this->localAgentID)) {
        this->engine->getTeamManager()->setTimeLastMsgReceived(aai->senderID, now);
        if (*(aai->senderID) > *(this->localAgentID)) {
            // notify TO that evidence about other robots is available
            for (EntryPointRobots epr : aai->entryPointRobots) {
                for (auto& rid : epr.robots) {
                    if (*rid != *(this->localAgentID)) {
                        this->engine->getTeamManager()->setTimeLastMsgReceived(rid, now);
                    }
                }
            }
        }
    }
#ifdef AM_DEBUG
    stringstream ss;
    ss << "AM: Received AAI Assignment from " << aai->senderID << " is: " << endl;
    for (EntryPointRobots epRobots : aai->entryPointRobots) {
        ss << "EP: " << epRobots.entrypoint << " Robots: ";
        for (int robot : epRobots.robots) {
            ss << robot << ", ";
        }
        ss << endl;
    }
    cout << ss.str();
#endif
    {
        lock_guard<mutex> lock(mu);
        this->queue.push_back(aai);
    }
}

/**
 * Cyclic tick function, called by the plan base every iteration
 */
void AuthorityManager::tick(shared_ptr<RunningPlan> rp) {
#ifdef AM_DEBUG
    cout << "AM: Tick called! <<<<<<" << endl;
#endif
    lock_guard<mutex> lock(mu);
    processPlan(rp);
    this->queue.clear();
}

void AuthorityManager::processPlan(shared_ptr<RunningPlan> rp) {
    if (rp == nullptr || rp->isBehaviour()) {
        return;
    }
    if (rp->getCycleManagement()->needsSending()) {
        sendAllocation(rp);
        rp->getCycleManagement()->sent();
    }
#ifdef AM_DEBUG
    cout << "AM: Queue size of AuthorityInfos is " << this->queue.size() << endl;
#endif
    for (int i = 0; i < this->queue.size(); i++) {
        if (authorityMatchesPlan(this->queue[i], rp)) {
#ifdef AM_DEBUG
            cout << "AM: Found AuthorityInfo, which matches the plan " << rp->getPlan()->getName() << endl;
#endif
            rp->getCycleManagement()->handleAuthorityInfo(this->queue[i]);
            this->queue.erase(this->queue.begin() + i);
            i--;
        }
    }
    for (shared_ptr<RunningPlan> c : *rp->getChildren()) {
        processPlan(c);
    }
}
/**
 * Sends an AllocationAuthorityInfo message containing the assignment of p
 */
void AuthorityManager::sendAllocation(shared_ptr<RunningPlan> p) {
    if (!this->engine->isMaySendMessages()) {
        return;
    }
    AllocationAuthorityInfo aai = AllocationAuthorityInfo();

    shared_ptr<Assignment> ass = p->getAssignment();
    for (int i = 0; i < ass->getEntryPointCount(); i++) {
        EntryPointRobots epRobots;
        epRobots.entrypoint = ass->getEpRobotsMapping()->getEp(i)->getId();
        for (auto robot : *ass->getRobotsWorking(epRobots.entrypoint)) {
            epRobots.robots.push_back(robot);
        }
        aai.entryPointRobots.push_back(epRobots);
    }

    auto shared = p->getParent().lock();
    aai.parentState =
            ((p->getParent().expired() || shared->getActiveState() == nullptr) ? -1
                                                                               : shared->getActiveState()->getId());
    aai.planId = p->getPlan()->getId();
    aai.authority = this->localAgentID;
    aai.senderID = this->localAgentID;
    aai.planType = (p->getPlanType() == nullptr ? -1 : p->getPlanType()->getId());
#ifdef AM_DEBUG
    stringstream ss;
    ss << "AM: Sending AAI Assignment from " << aai.senderID << " is: " << endl;
    for (EntryPointRobots epRobots : aai.entryPointRobots) {
        ss << "EP: " << epRobots.entrypoint << " Robots: ";
        for (int robot : epRobots.robots) {
            ss << robot << ", ";
        }
        ss << endl;
    }
    cout << ss.str();
#endif
    this->engine->getCommunicator()->sendAllocationAuthority(aai);
}

/**
 * FIXME: Bug in authority manager
 * QUESTION: If you use the same plan two times, each in a different part of the plan tree.
 * How is it guarenteed, that this method does not match the wrong instance?!
 * @param aai
 * @param p
 * @return
 */
bool AuthorityManager::authorityMatchesPlan(shared_ptr<AllocationAuthorityInfo> aai, shared_ptr<RunningPlan> p) {
    auto shared = p->getParent().lock();
    /*#ifdef AM_DEBUG
                    if (!p->getParent().expired())
                    {
                            cout << "AM: Parent-WeakPtr is NOT expired!" << endl;
                            cout << "AM: Parent-ActiveState is: " << (shared->getActiveState() != nullptr ?
    shared->getActiveState()->getId() : NULL) << endl; cout << "AM: AAI-ParentState is: " << aai->parentState << endl;
                    }
                    else
                    {
                            cout << "AM: Parent-WeakPtr is expired!" << endl;
                            cout << "AM: Current-ActiveState is: " << p->getActiveState()->getId() << endl;
                            cout << "AM: AAI-ParentState is: " << aai->parentState << endl;
                    }
    #endif*/

    if ((p->getParent().expired() && aai->parentState == -1) ||
            (!p->getParent().expired() && shared->getActiveState() != nullptr &&
                    shared->getActiveState()->getId() == aai->parentState)) {
        if (p->getPlan()->getId() == aai->planId) {
            return true;
        } else if (aai->planType != -1 && p->getPlanType() != nullptr && p->getPlanType()->getId() == aai->planType) {
            return true;
        }
    }
    return false;
}

}  // namespace alica
