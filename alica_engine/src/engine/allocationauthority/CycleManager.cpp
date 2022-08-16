#include "engine/allocationauthority/CycleManager.h"

#include "engine/AlicaClock.h"
#include "engine/Assignment.h"
#include "engine/ConfigChangeListener.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/Types.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/containers/EntryPointRobots.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/logging/Logging.h"

#include <functional>

namespace alica
{
using std::lock_guard;
using std::make_shared;
using std::mutex;

/**
 * Construct a CycleManager for a RunningPlan
 * @param p A RunningPlan
 */
CycleManager::CycleManager(ConfigChangeListener& configChangeListener, const AlicaClock& clock, const TeamManager& teamManager,
        const PlanRepository& planRepository, RunningPlan* rp)
        : _state(CycleState::observing)
        , _configChangeListener(configChangeListener)
        , _clock(clock)
        , _teamManager(teamManager)
        , _planRepository(planRepository)
        , _fixedAllocation()
        , _newestAllocationDifference(0)
        , _rp(rp)
        , _myID(_teamManager.getLocalAgentID())

{
    auto reloadFunctionPtr = std::bind(&CycleManager::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_configChangeListener.getConfig());
}

CycleManager::~CycleManager() {}

void CycleManager::reload(const YAML::Node& config)
{
    _maxAllocationCycles = config["Alica"]["CycleDetection"]["CycleCount"].as<int>();
    _enabled = config["Alica"]["CycleDetection"]["Enabled"].as<bool>();
    _minimalOverrideTimeInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MinimalAuthorityTimeInterval"].as<unsigned long>());
    _maximalOverrideTimeInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MaximalAuthorityTimeInterval"].as<unsigned long>());
    _overrideShoutInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MessageTimeInterval"].as<unsigned long>());
    _overrideWaitInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MessageWaitTimeInterval"].as<unsigned long>());
    int historySize = config["Alica"]["CycleDetection"]["HistorySize"].as<int>();

    _intervalIncFactor = config["Alica"]["CycleDetection"]["IntervalIncreaseFactor"].as<double>();
    _intervalDecFactor = config["Alica"]["CycleDetection"]["IntervalDecreaseFactor"].as<double>();

    _allocationHistory.resize(historySize);
}

/**
 * Called once per engine iteration by the corresponding RunningPlan.
 * Checks whether a state change is needed.
 */
void CycleManager::update()
{
    if (!_enabled) {
        return;
    }
    if (_rp->isBehaviour()) {
        return;
    }

    const Plan* plan = dynamic_cast<const Plan*>(_rp->getActivePlan());

    if (_state == CycleState::observing) {
        if (detectAllocationCycle()) {
            Logging::logInfo("CM") << "Cycle Detected!";

            _state = CycleState::overriding;
            plan->setAuthorityTimeInterval(std::min(_maximalOverrideTimeInterval, plan->getAuthorityTimeInterval() * _intervalIncFactor));
            _overrideShoutTime = AlicaTime::zero();
            Logging::logDebug("CM") << "Assuming Authority for " << plan->getAuthorityTimeInterval().inSeconds() << "sec!";
            _overrideTimestamp = _clock.now();
        } else {
            plan->setAuthorityTimeInterval(std::max(_minimalOverrideTimeInterval, plan->getAuthorityTimeInterval() * _intervalDecFactor));
        }
    } else {
        if (_state == CycleState::overriding && _overrideTimestamp + plan->getAuthorityTimeInterval() < _clock.now()) {
            Logging::logDebug("CM") << "Resume Observing!";
            _state = CycleState::observing;
            _fixedAllocation = AllocationAuthorityInfo();
        } else if (_state == CycleState::overridden && _overrideShoutTime + plan->getAuthorityTimeInterval() < _clock.now()) {
            Logging::logDebug("CM") << "Resume Observing!";
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
    return _state == CycleState::overridden && _fixedAllocation.authority != InvalidAgentID;
}

AllocationDifference& CycleManager::editNextDifference()
{
    _newestAllocationDifference = (_newestAllocationDifference + 1) % _allocationHistory.size();
    _allocationHistory[_newestAllocationDifference].reset();
    return _allocationHistory[_newestAllocationDifference];
}
/**
 * Notify the CycleManager of a change in the assignment
 * @param oldAss The former Assignment
 * @param newAss The new Assignment
 * @param reas The AllocationDifference.Reason for this change.
 */
void CycleManager::setNewAllocDiff(const Assignment& oldAss, const Assignment& newAss, AllocationDifference::Reason reas)
{
    if (!_enabled) {
        return;
    }

    _newestAllocationDifference = (_newestAllocationDifference + 1) % _allocationHistory.size();

    AllocationDifference& aldif = _allocationHistory[_newestAllocationDifference];
    aldif.reset();
    const int oldEpCount = oldAss.getEntryPointCount();
    if (newAss.getPlan() == oldAss.getPlan()) {
        for (int i = 0; i < oldEpCount; ++i) {
            const AgentStatePairs& newAgents = newAss.getAgentStates(i);
            const AgentStatePairs& oldAgents = oldAss.getAgentStates(i);

            for (AgentStatePair oldp : oldAgents) {
                if (!newAgents.hasAgent(oldp.first)) {
                    aldif.editSubtractions().emplace_back(newAss.getEntryPoint(i), oldp.first);
                }
            }
            for (AgentStatePair newp : newAgents) {
                if (!oldAgents.hasAgent(newp.first)) {
                    aldif.editAdditions().emplace_back(newAss.getEntryPoint(i), newp.first);
                }
            }
        }
    } else {
        for (int i = 0; i < oldEpCount; ++i) {
            for (AgentStatePair oldp : oldAss.getAgentStates(i)) {
                aldif.editSubtractions().emplace_back(oldAss.getEntryPoint(i), oldp.first);
            }
        }
        for (int i = 0; i < newAss.getEntryPointCount(); ++i) {
            for (AgentStatePair newp : newAss.getAgentStates(i)) {
                aldif.editAdditions().emplace_back(newAss.getEntryPoint(i), newp.first);
            }
        }
    }
    aldif.setReason(reas);
    Logging::logDebug("CM") << "SetNewAllDiff(b): " << aldif;
}

/**
 * Message Handler
 * @param aai A shared_ptr<AllocationAuthorityInfo>
 */
void CycleManager::handleAuthorityInfo(const AllocationAuthorityInfo& aai)
{
    if (!_enabled) {
        return;
    }

    if (_rp->isBehaviour()) {
        return;
    }

    AgentId rid = aai.authority;
    if (rid == _myID) {
        return;
    }
    if (rid < _myID) {
        Logging::logInfo("CM") << "Rcv: Rejecting Authority!";
        if (_state != CycleState::overriding) {
            Logging::logDebug("CM") << "Overriding assignment of " << _rp->getActivePlan()->getName();

            _state = CycleState::overriding;
            const Plan* plan = dynamic_cast<const Plan*>(_rp->getActivePlan());
            plan->setAuthorityTimeInterval(std::min(_maximalOverrideTimeInterval, (plan->getAuthorityTimeInterval() * _intervalIncFactor)));
            _overrideTimestamp = _clock.now();
            _overrideShoutTime = AlicaTime::zero();
        }
    } else {
        Logging::logDebug("CM") << "Assignment overridden in " << _rp->getActivePlan()->getName();
        _state = CycleState::overridden;
        _overrideShoutTime = _clock.now();
        _fixedAllocation = aai;
    }
}

bool CycleManager::needsSending() const
{
    return _state == CycleState::overriding && (_overrideShoutTime + _overrideShoutInterval < _clock.now());
}

/**
 * Indicate to the manager that a corresponding message has been sent.
 */
void CycleManager::sent()
{
    _overrideShoutTime = _clock.now();
}

/**
 * Apply the authorative assignment to the RunningPlan
 * @param r A shared_ptr<RunningPlan>
 * @return A bool
 */
bool CycleManager::applyAssignment()
{
    Logging::logDebug("CM") << "Setting authorative assignment for plan " << _rp->getActivePlan()->getName();

    if (_fixedAllocation.authority == InvalidAgentID) {
        return false;
    }
    const EntryPoint* myEntryPoint = nullptr;
    bool modifiedSelf = false;
    bool modified = false;
    if (_fixedAllocation.planId != _rp->getActivePlan()->getId()) { // Plantype case
        if (_rp->getPlanType()->getId() != _fixedAllocation.planType) {
            return false;
        }
        const Plan* newPlan = _rp->getPlanType()->getPlanById(_fixedAllocation.planId);
        assert(newPlan != nullptr);
        _rp->usePlan(newPlan);
        _rp->setAssignment(Assignment(newPlan, _fixedAllocation));
        myEntryPoint = _rp->getAssignment().getEntryPointOfAgent(_myID);
        modifiedSelf = true;
    } else {
        for (EntryPointRobots epr : _fixedAllocation.entryPointRobots) {
            for (AgentId robot : epr.robots) {
                const EntryPoint* e = _planRepository.getEntryPoints()[epr.entrypoint];
                bool changed = _rp->editAssignment().updateAgent(robot, e);
                if (changed) {
                    if (robot == _myID) {
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
        _rp->useEntryPoint(myEntryPoint);
        _rp->deactivateChildren();
        _rp->clearChildren();
        _rp->clearFailedChildren();
        _rp->setAllocationNeeded(true);
    } else {
        if (_rp->getActiveState() != nullptr) {
            AgentGrp robotsJoined;
            _rp->getAssignment().getAgentsInState(_rp->getActiveState(), robotsJoined);
            for (RunningPlan* c : _rp->getChildren()) {
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

    for (int i = _newestAllocationDifference; count < static_cast<int>(_allocationHistory.size()); --i) {
        ++count;
        if (i < 0) {
            i = _allocationHistory.size() - 1;
        }

        if (_allocationHistory[i].getReason() == AllocationDifference::Reason::utility) {
            if (utChange != nullptr) {
                return false;
            }
            utChange = &_allocationHistory[i];
            temp.reset();
            temp.applyDifference(*utChange);
        } else {
            if (_allocationHistory[i].getReason() == AllocationDifference::Reason::empty) {
                return false;
            }
            if (utChange == nullptr) {
                continue;
            }
            temp.applyDifference(_allocationHistory[i]);
            if (temp.isEmpty()) {
                ++cyclesFound;
                if (cyclesFound > _maxAllocationCycles) {
                    for (int k = 0; k < static_cast<int>(_allocationHistory.size()); ++k) {
                        _allocationHistory[k].reset();
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
