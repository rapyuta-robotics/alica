#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/Assignment.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"

namespace alica
{
/**
 * Constructor
 */
AuthorityManager::AuthorityManager(AlicaEngine* engine)
    : engine(engine)
    , localAgentID(nullptr)
{
}

AuthorityManager::~AuthorityManager() {}

/**
 * Initialises this engine module
 */
void AuthorityManager::init()
{
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
void AuthorityManager::handleIncomingAuthorityMessage(shared_ptr<AllocationAuthorityInfo> aai)
{
    auto now = this->engine->getAlicaClock()->now();
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
    std::stringstream ss;
    ss << "AM: Received AAI Assignment from " << aai->senderID << " is: " << std::endl;
    for (EntryPointRobots epRobots : aai->entryPointRobots) {
        ss << "EP: " << epRobots.entrypoint << " Robots: ";
        for (int robot : epRobots.robots) {
            ss << robot << ", ";
        }
        ss << std::endl;
    }
    cout << ss.str();
#endif
    {
        std::lock_guard<std::mutex> lock(mu);
        this->queue.push_back(aai);
    }
}

/**
 * Cyclic tick function, called by the plan base every iteration
 */
void AuthorityManager::tick(std::shared_ptr<RunningPlan> rp)
{
#ifdef AM_DEBUG
    std::cout << "AM: Tick called! <<<<<<" << std::endl;
#endif
    std::lock_guard<std::mutex> lock(mu);
    processPlan(rp);
    this->queue.clear();
}

void AuthorityManager::processPlan(shared_ptr<RunningPlan> rp)
{
    if (rp == nullptr || rp->isBehaviour()) {
        return;
    }
    if (rp->getCycleManagement()->needsSending()) {
        sendAllocation(rp);
        rp->getCycleManagement()->sent();
    }
#ifdef AM_DEBUG
    std::cout << "AM: Queue size of AuthorityInfos is " << this->queue.size() << std::endl;
#endif
    for (int i = 0; i < static_cast<int>(this->queue.size()); ++i) {
        if (authorityMatchesPlan(this->queue[i], rp)) {
#ifdef AM_DEBUG
            std::cout << "AM: Found AuthorityInfo, which matches the plan " << rp->getPlan()->getName() << std::endl;
#endif
            rp->getCycleManagement()->handleAuthorityInfo(this->queue[i]);
            this->queue.erase(this->queue.begin() + i);
            i--;
        }
    }
    for (std::shared_ptr<RunningPlan>& c : *rp->getChildren()) {
        processPlan(c);
    }
}
/**
 * Sends an AllocationAuthorityInfo message containing the assignment of p
 */
void AuthorityManager::sendAllocation(std::shared_ptr<RunningPlan> p)
{
    if (!this->engine->maySendMessages()) {
        return;
    }
    AllocationAuthorityInfo aai = AllocationAuthorityInfo();

    std::shared_ptr<Assignment> ass = p->getAssignment();
    for (int i = 0; i < ass->getEntryPointCount(); i++) {
        EntryPointRobots epRobots;
        epRobots.entrypoint = ass->getEpRobotsMapping()->getEp(i)->getId();
        for (auto robot : *ass->getRobotsWorking(epRobots.entrypoint)) {
            epRobots.robots.push_back(robot);
        }
        aai.entryPointRobots.push_back(epRobots);
    }

    auto shared = p->getParent().lock();
    aai.parentState = ((p->getParent().expired() || shared->getActiveState() == nullptr) ? -1 : shared->getActiveState()->getId());
    aai.planId = p->getPlan()->getId();
    aai.authority = this->localAgentID;
    aai.senderID = this->localAgentID;
    aai.planType = (p->getPlanType() == nullptr ? -1 : p->getPlanType()->getId());
#ifdef AM_DEBUG
    std::stringstream ss;
    ss << "AM: Sending AAI Assignment from " << aai.senderID << " is: " << std::endl;
    for (EntryPointRobots epRobots : aai.entryPointRobots) {
        ss << "EP: " << epRobots.entrypoint << " Robots: ";
        for (int robot : epRobots.robots) {
            ss << robot << ", ";
        }
        ss << std::endl;
    }
    std::cout << ss.str();
#endif
    this->engine->getCommunicator()->sendAllocationAuthority(aai);
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
        (parent != nullptr && parent->getActiveState() != nullptr && parent->getActiveState()->getId() == aai->parentState)) {
        if (p.getActivePlan()->getId() == aai->planId) {
            return true;
        } else if (aai->planType != -1 && p.getPlanType() != nullptr && p.getPlanType()->getId() == aai->planType) {
            return true;
        }
    }
    return false;
}

} // namespace alica
