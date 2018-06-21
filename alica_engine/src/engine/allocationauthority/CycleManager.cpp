#include "engine/allocationauthority/CycleManager.h"

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/Types.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/containers/EntryPointRobots.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <SystemConfig.h>

#include <alica_common_config/debug_output.h>

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
        : _state(CycleState::observing)
        , _ae(ae)
        , _fixedAllocation()
{
    sc = supplementary::SystemConfig::getInstance();
    maxAllocationCycles = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "CycleCount");
    enabled = (*sc)["Alica"]->get<bool>("Alica", "CycleDetection", "Enabled");
    minimalOverrideTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL));
    maximalOverrideTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MaximalAuthorityTimeInterval", NULL));
    overrideShoutInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MessageTimeInterval", NULL));
    overrideWaitInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MessageWaitTimeInterval", NULL));
    historySize = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "HistorySize", NULL);

    this->intervalIncFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalIncreaseFactor", NULL);
    this->intervalDecFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalDecreaseFactor", NULL);

    this->allocationHistory.resize(this->historySize);
    for (int i = 0; i < this->historySize; i++) {
        this->allocationHistory[i] = new AllocationDifference();
    }
    this->newestAllocationDifference = 0;

    this->rp = p;
    this->myID = ae->getTeamManager()->getLocalAgentID();
    this->pr = ae->getPlanRepository();
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

    const AbstractPlan* plan = this->rp->getActivePlan();

    if (_state == CycleState::observing) {
        if (detectAllocationCycle()) {
            ALICA_INFO_MSG("CM: Cycle Detected!");

            _state = CycleState::overriding;
            plan->setAuthorityTimeInterval(std::min(maximalOverrideTimeInterval, plan->getAuthorityTimeInterval() * intervalIncFactor));
            this->overrideShoutTime = AlicaTime::zero();

            ALICA_DEBUG_MSG("CM: Assuming Authority for " << plan->getAuthorityTimeInterval().inSeconds() << "sec!");
            this->overrideTimestamp = _ae->getAlicaClock()->now();
        } else {
            plan->setAuthorityTimeInterval(std::max(minimalOverrideTimeInterval, plan->getAuthorityTimeInterval() * intervalDecFactor));
        }
    } else {
        if (_state == CycleState::overriding && this->overrideTimestamp + plan->getAuthorityTimeInterval() < _ae->getAlicaClock()->now()) {
            ALICA_DEBUG_MSG("CM: Resume Observing!");
            _state = CycleState::observing;
            _fixedAllocation = AllocationAuthorityInfo();
        } else if (_state == CycleState::overridden && this->overrideShoutTime + plan->getAuthorityTimeInterval() < _ae->getAlicaClock()->now()) {
            ALICA_DEBUG_MSG("CM: Resume Observing!");
            _state = CycleState::observing;
            _fixedAllocation = AllocationAuthorityInfo();
        }
    }
}

/**
 * Indicates whether an authorative allocation is present
 * @return bool
 */
bool CycleManager::isOverridden() const
{
    return _state == CycleState::overridden && _fixedAllocation.authority != nullptr;
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

    ALICA_DEBUG_MSG("CM: SetNewAllDiff(a): " << aldif->toString() << " OWN ROBOT ID " << *(this->rp->getOwnID()));
}

/**
 * Notify the CycleManager of a change in the assignment
 * @param oldAss The former Assignment
 * @param newAss The new Assignment
 * @param reas The AllocationDifference.Reason for this change.
 */
void alica::CycleManager::setNewAllocDiff(const Assignment& oldAss, const Assignment& newAss, AllocationDifference::Reason reas)
{
    if (!enabled) {
        return;
    }

    std::lock_guard<mutex> lock(this->allocationHistoryMutex);

    this->newestAllocationDifference = (this->newestAllocationDifference + 1) % this->allocationHistory.size();
    this->allocationHistory[this->newestAllocationDifference]->reset();
    const int oldEpCount = oldAss.getEntryPointCount();
    if (newAss.getPlan() == oldAss.getPlan()) {
        for (int i = 0; i < oldEpCount; ++i) {
            const AgentStatePairs& newAgents = newAss.getAgentStates(i);
            const AgentStatePairs& oldAgents = oldAss.getAgentStates(i);

            for (AgentStatePair oldp : oldAgents) {
                if (!newAgents.hasAgent(oldp.first)) {
                    this->allocationHistory[this->newestAllocationDifference]->editSubtractions().emplace_back(newAss.getEntryPoint(i), oldp.first);
                }
            }
            for (AgentStatePair newp : newAgents) {
                if (!oldAgents.hasAgent(newp.first)) {
                    this->allocationHistory[this->newestAllocationDifference]->editAdditions().emplace_back(newAss.getEntryPoint(i), newp.first);
                }
            }
        }
    } else {
        for (int i = 0; i < oldEpCount; ++i) {
            for (AgentStatePair oldp : oldAss.getAgentStates(i)) {
                this->allocationHistory[this->newestAllocationDifference]->editSubtractions().emplace_back(oldAss.getEntryPoint(i), oldp.first);
            }
        }
        for (int i = 0; i < newAss.getEntryPointCount(); ++i) {
            for (AgentStatePair newp : newAss.getAgentStates(i)) {
                this->allocationHistory[this->newestAllocationDifference]->editAdditions().emplace_back(newAss.getEntryPoint(i), newp.first);
            }
        }
    }
    this->allocationHistory[this->newestAllocationDifference]->setReason(reas);
    ALICA_DEBUG_MSG("CM: SetNewAllDiff(b): " << this->allocationHistory[this->newestAllocationDifference]->toString());
}

/**
 * Message Handler
 * @param aai A shared_ptr<AllocationAuthorityInfo>
 */
void alica::CycleManager::handleAuthorityInfo(const AllocationAuthorityInfo& aai)
{
    if (!enabled) {
        return;
    }
    AgentIDConstPtr rid = aai.authority;
    if (*rid == *myID) {
        return;
    }
    if (*rid > *myID) {
        ALICA_DEBUG_MSG("CM: Assignment overridden in " << this->rp->getActivePlan()->getName());
        _state = CycleState::overridden;
        this->overrideShoutTime = _ae->getAlicaClock()->now();
        _fixedAllocation = aai;
    } else {
        std::cout << "CM: Rcv: Rejecting Authority!" << std::endl;
        if (_state != CycleState::overriding) {
            ALICA_DEBUG_MSG("CM: Overriding assignment of " << this->rp->getActivePlan()->getName());

            _state = CycleState::overriding;
            this->rp->getActivePlan()->setAuthorityTimeInterval(
                    std::min(maximalOverrideTimeInterval, (this->rp->getActivePlan()->getAuthorityTimeInterval() * intervalIncFactor)));
            this->overrideTimestamp = _ae->getAlicaClock()->now();
            this->overrideShoutTime = AlicaTime::zero();
        }
    }
}

bool alica::CycleManager::needsSending() const
{
    return _state == CycleState::overriding && (this->overrideShoutTime + overrideShoutInterval < _ae->getAlicaClock()->now());
}

/**
 * Indicate to the manager that a corresponding message has been sent.
 */
void alica::CycleManager::sent()
{
    this->overrideShoutTime = _ae->getAlicaClock()->now();
}

/**
 * Apply the authorative assignment to the RunningPlan
 * @param r A shared_ptr<RunningPlan>
 * @return A bool
 */
bool CycleManager::applyAssignment()
{
    ALICA_DEBUG_MSG("CM: Setting authorative assignment for plan " << rp->getActivePlan()->getName());

    if (_fixedAllocation.authority == nullptr) {
        return false;
    }
    const EntryPoint* myEntryPoint = nullptr;
    bool modifiedSelf = false;
    bool modified = false;
    if (_fixedAllocation.planId != rp->getActivePlan()->getId()) { // Plantype case
        if (rp->getPlanType()->getId() != _fixedAllocation.planType) {
            return false;
        }
        const Plan* newPlan = nullptr;
        for (const Plan* p : rp->getPlanType()->getPlans()) {
            if (p->getId() == _fixedAllocation.planId) {
                newPlan = p;
                rp->usePlan(p);
                break;
            }
        }
        rp->setAssignment(Assignment(newPlan, _fixedAllocation));
        myEntryPoint = rp->getAssignment().getEntryPointOfAgent(myID);
        modifiedSelf = true;
    } else {
        for (EntryPointRobots epr : _fixedAllocation.entryPointRobots) {
            for (AgentIDConstPtr robot : epr.robots) {
                const EntryPoint* e = pr->getEntryPoints()[epr.entrypoint];
                bool changed = rp->editAssignment().updateAgent(robot, e);
                if (changed) {
                    if (robot == myID) {
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
        rp->useEntryPoint(myEntryPoint);
        rp->deactivateChildren();
        rp->clearChildren();
        rp->clearFailedChildren();
        rp->setAllocationNeeded(true);
    } else {
        if (rp->getActiveState() != nullptr) {
            AgentGrp robotsJoined;
            rp->getAssignment().getAgentsInState(rp->getActiveState(), robotsJoined);
            for (RunningPlan* c : rp->getChildren()) {
                c->limitToRobots(robotsJoined);
            }
        }
    }
    return modifiedSelf || modified;
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
