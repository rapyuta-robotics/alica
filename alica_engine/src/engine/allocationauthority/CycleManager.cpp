#include "engine/allocationauthority/CycleManager.h"

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/StateCollection.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/EntryPointRobots.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <SystemConfig.h>

namespace alica
{
using std::lock_guard;
using std::make_shared;
using std::mutex;

/**
 * Construct a CycleManager for a RunningPlan
 * @param p A RunningPlan
 */
CycleManager::CycleManager(AlicaEngine* ae, RunningPlan* p)
{
    sc = supplementary::SystemConfig::getInstance();
    maxAllocationCycles = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "CycleCount");
    enabled = (*sc)["Alica"]->get<bool>("Alica", "CycleDetection", "Enabled");
    minimalOverrideTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL));
    maximalOverrideTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MaximalAuthorityTimeInterval", NULL));
    overrideShoutInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MessageTimeInterval", NULL));
    overrideWaitInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MessageWaitTimeInterval", NULL));
    historySize = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "HistorySize", NULL);

    this->ae = ae;
    this->intervalIncFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalIncreaseFactor", NULL);
    this->intervalDecFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalDecreaseFactor", NULL);

    this->allocationHistory.resize(this->historySize);
    for (int i = 0; i < this->historySize; i++) {
        this->allocationHistory[i] = new AllocationDifference();
    }
    this->newestAllocationDifference = 0;
    this->state = CycleState::observing;
    this->rp = p;
    this->myID = ae->getTeamManager()->getLocalAgentID();
    this->pr = ae->getPlanRepository();
    this->fixedAllocation = nullptr;
}

CycleManager::~CycleManager()
{
    lock_guard<mutex> lock(this->allocationHistoryMutex);
    for (int i = 0; i < static_cast<int>(this->allocationHistory.size()); ++i) {
        delete this->allocationHistory[i];
    }
}

/**
 * Called once per engine iteration by the corresponding RunningPlan.
 * Checks whether a state change is needed.
 */
void CycleManager::update()
{
    if (!this->enabled) {
        return;
    }
    if (this->rp->isBehaviour()) {
        return;
    }

    const AbstractPlan* plan = this->rp->getPlan();

    if (this->state == CycleState::observing) {
        if (detectAllocationCycle()) {
            std::cout << "CM: Cycle Detected!" << std::endl;

            this->state = CycleState::overriding;
            plan->setAuthorityTimeInterval(std::min(maximalOverrideTimeInterval, plan->getAuthorityTimeInterval() * intervalIncFactor));
            this->overrideShoutTime = AlicaTime::zero();
#ifdef CM_DEBUG
            std::cout << "Assuming Authority for " << plan->getAuthorityTimeInterval().inSeconds() << "sec!" << std::endl;
#endif
            this->overrideTimestamp = ae->getAlicaClock()->now();
        } else {
            plan->setAuthorityTimeInterval(std::max(minimalOverrideTimeInterval, plan->getAuthorityTimeInterval() * intervalDecFactor));
        }
    } else {
        if (this->state == CycleState::overriding && this->overrideTimestamp + plan->getAuthorityTimeInterval() < ae->getAlicaClock()->now()) {
#ifdef CM_DEBUG
            std::cout << "Resume Observing!" << std::endl;
#endif
            this->state = CycleState::observing;
            this->fixedAllocation = nullptr;
        } else if (this->state == CycleState::overridden && this->overrideShoutTime + plan->getAuthorityTimeInterval() < ae->getAlicaClock()->now()) {
#ifdef CM_DEBUG
            std::cout << "Resume Observing!" << std::endl;
#endif
            this->state = CycleState::observing;
            this->fixedAllocation = nullptr;
        }
    }
}

/**
 * Indicates whether an authorative allocation is present
 * @return bool
 */
bool CycleManager::isOverridden() const
{
    return this->state == CycleState::overridden && this->fixedAllocation != nullptr;
}

/**
 * Notify the CycleManager of a new AllocationDifference
 * @param curP The RunningPlan of this CycleManager, in case it has changed.
 * @param aldif The new AllocationDifference
 */
void alica::CycleManager::setNewAllocDiff(AllocationDifference* aldif)
{
    if (!enabled) {
        return;
    }
    lock_guard<mutex> lock(this->allocationHistoryMutex);

    this->newestAllocationDifference = (this->newestAllocationDifference + 1) % this->allocationHistory.size();
    AllocationDifference* old = this->allocationHistory[this->newestAllocationDifference];
    this->allocationHistory[this->newestAllocationDifference] = aldif;
    delete old;
#ifdef CM_DEBUG
    std::cout << "CM: SetNewAllDiff(a): " << aldif->toString() << " OWN ROBOT ID " << *(this->rp->getOwnID()) << std::endl;
#endif
}

/**
 * Notify the CycleManager of a change in the assignment
 * @param oldAss The former Assignment
 * @param newAss The new Assignment
 * @param reas The AllocationDifference.Reason for this change.
 */
void alica::CycleManager::setNewAllocDiff(shared_ptr<Assignment> oldAss, shared_ptr<Assignment> newAss, AllocationDifference::Reason reas)
{
    if (!enabled) {
        return;
    }
    if (oldAss == nullptr) {
        return;
    }
    lock_guard<mutex> lock(this->allocationHistoryMutex);
    try {
        this->newestAllocationDifference = (this->newestAllocationDifference + 1) % this->allocationHistory.size();
        this->allocationHistory[this->newestAllocationDifference]->reset();

        // for (EntryPoint* ep : (*oldAss->getEntryPoints()))
        for (short i = 0; i < oldAss->getEntryPointCount(); i++) {
            const EntryPoint* ep = oldAss->getEpRobotsMapping()->getEp(i);

            auto newRobots = newAss->getRobotsWorking(ep);
            auto oldRobots = oldAss->getRobotsWorking(ep);
            for (auto& oldId : (*oldRobots)) {
                if (newRobots == nullptr ||
                    find_if(newRobots->begin(), newRobots->end(), [&oldId](const supplementary::AgentID* id) { return *oldId == *id; }) == newRobots->end()) {
                    this->allocationHistory[this->newestAllocationDifference]->editSubtractions().emplace_back(ep, oldId);
                }
            }
            if (newRobots != nullptr) {
                for (auto& newId : (*newRobots)) {
                    if (find_if(oldRobots->begin(), oldRobots->end(), [&newId](const supplementary::AgentID* id) { return *newId == *id; }) ==
                        oldRobots->end()) {
                        this->allocationHistory[this->newestAllocationDifference]->editAdditions().emplace_back(ep, newId);
                    }
                }
            }
        }
        this->allocationHistory[this->newestAllocationDifference]->setReason(reas);
#ifdef CM_DEBUG
        std::cout << "CM: SetNewAllDiff(b): " << this->allocationHistory[this->newestAllocationDifference]->toString() << std::endl;
#endif
    } catch (std::exception& e) {
        std::cerr << "Exception in Alloc Difference Calculation:" << std::endl;
        std::cerr << e.what();
    }
}

/**
 * Message Handler
 * @param aai A shared_ptr<AllocationAuthorityInfo>
 */
void alica::CycleManager::handleAuthorityInfo(shared_ptr<AllocationAuthorityInfo> aai)
{
    if (!enabled) {
        return;
    }
    auto rid = aai->authority;
    if (*rid == *myID) {
        return;
    }
    if (*rid > *myID) {
#ifdef CM_DEBUG
        std::cout << "CM: Assignment overridden in " << this->rp->getPlan()->getName() << " " << std::endl;
#endif
        this->state = CycleState::overridden;
        this->overrideShoutTime = ae->getAlicaClock()->now();
        this->fixedAllocation = aai;
    } else {
        std::cout << "CM: Rcv: Rejecting Authority!" << std::endl;
        if (this->state != CycleState::overriding) {
#ifdef CM_DEBUG
            std::cout << "CM: Overriding assignment of " << this->rp->getPlan()->getName() << " " << std::endl;
#endif
            this->state = CycleState::overriding;
            this->rp->getPlan()->setAuthorityTimeInterval(
                std::min(maximalOverrideTimeInterval, (this->rp->getPlan()->getAuthorityTimeInterval() * intervalIncFactor)));
            this->overrideTimestamp = ae->getAlicaClock()->now();
            this->overrideShoutTime = AlicaTime::zero();
        }
    }
}

bool alica::CycleManager::needsSending()
{
    return this->state == CycleState::overriding && (this->overrideShoutTime + overrideShoutInterval < ae->getAlicaClock()->now());
}

/**
 * Indicate to the manager that a corresponding message has been sent.
 */
void alica::CycleManager::sent()
{
    this->overrideShoutTime = ae->getAlicaClock()->now();
}

/**
 * Indicates whether the local agent currently holds authority over the plan.
 * @return A bool
 */
bool alica::CycleManager::haveAuthority()
{
    return this->state == CycleState::overriding;
}

/**
 * Apply the authorative assignment to the RunningPlan
 * @param r A shared_ptr<RunningPlan>
 * @return A bool
 */
bool CycleManager::setAssignment()
{
#ifdef CM_DEBUG
    std::cout << "CM: Setting authorative assignment for plan " << rp->getPlan()->getName() << std::endl;
    if (rp->getPlan()->getName() == "AuthorityTest") {
        std::cout << "CM: Changing AuthorityTest " << std::endl;
    }
#endif
    const EntryPoint* myEntryPoint = nullptr;
    if (this->fixedAllocation == nullptr) {
        return false;
    }
    bool modifiedSelf = false;
    bool modified = false;
    if (this->fixedAllocation->planId != rp->getPlan()->getId()) { // Plantype case
        if (rp->getPlanType()->getId() != this->fixedAllocation->planType) {
            return false;
        }
        const Plan* newPlan = nullptr;
        for (const Plan* p : rp->getPlanType()->getPlans()) {
            if (p->getId() == this->fixedAllocation->planId) {
                newPlan = p;
                rp->setPlan(p);
                break;
            }
        }
        rp->setAssignment(make_shared<Assignment>(newPlan, this->fixedAllocation));
        for (const EntryPointRobots& epr : this->fixedAllocation->entryPointRobots) {
            if (find_if(epr.robots.begin(), epr.robots.end(), [this](const supplementary::AgentID* id) { return *(this->myID) == *id; }) != epr.robots.end()) {
                myEntryPoint = pr->getEntryPoints()[epr.entrypoint];
            }
        }

        modifiedSelf = true;
    } else {
        for (EntryPointRobots epr : this->fixedAllocation->entryPointRobots) {
            for (auto& robot : epr.robots) {
                const EntryPoint* e = pr->getEntryPoints()[epr.entrypoint];
                bool changed = rp->getAssignment()->updateRobot(robot, e);
                if (changed) {
                    if (*robot == *myID) {
                        modifiedSelf = true;
                        myEntryPoint = e;
                    } else {
                        modified = true;
                    }
                }
            }
        }
    }
    if (modifiedSelf) {
        rp->setOwnEntryPoint(myEntryPoint);
        rp->deactivateChildren();
        rp->clearChildren();
        rp->clearFailedChildren();
        rp->setAllocationNeeded(true);
    } else {
        if (rp->getActiveState() != nullptr) {
            AgentGrp robotsJoined;
            rp->getAssignment()->getRobotStateMapping()->getRobotsInState(rp->getActiveState(), robotsJoined);
            for (shared_ptr<RunningPlan>& c : *rp->getChildren()) {
                c->limitToRobots(robotsJoined);
            }
        }
    }
    return modifiedSelf || modified;
}

/**
 * Indicates wether authority allows the assignment of this plan to be changed.
 * @return A bool
 */
bool CycleManager::mayDoUtilityCheck()
{
    return this->state != CycleState::overridden;
}

bool CycleManager::detectAllocationCycle()
{
    // A Cycle occurs n-times,
    // Consists of 1 UtilityChange, m message update
    // after uc, allocation is same again (delta = 0)
    int cyclesFound = 0;
    int count = 0;
    AllocationDifference* utChange = nullptr;
    AllocationDifference temp;
    lock_guard<mutex> lock(this->allocationHistoryMutex);

    for (int i = this->newestAllocationDifference; count < static_cast<int>(this->allocationHistory.size()); --i) {
        ++count;
        if (i < 0) {
            i = this->allocationHistory.size() - 1;
        }

        if (this->allocationHistory[i]->getReason() == AllocationDifference::Reason::utility) {
            if (utChange != nullptr) {
                return false;
            }
            utChange = this->allocationHistory[i];
            temp.reset();
            temp.applyDifference(*utChange);
        } else {
            if (this->allocationHistory[i]->getReason() == AllocationDifference::Reason::empty) {
                return false;
            }
            if (utChange == nullptr) {
                continue;
            }
            temp.applyDifference(*this->allocationHistory[i]);
            if (temp.isEmpty()) {
                ++cyclesFound;
                if (cyclesFound > maxAllocationCycles) {
                    for (int k = 0; k < static_cast<int>(this->allocationHistory.size()); ++k) {
                        this->allocationHistory[k]->reset();
                    }
                    return true;
                }
                utChange = nullptr;
            }
        }
    }

    return false;
}

} // namespace alica
