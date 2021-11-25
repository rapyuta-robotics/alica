#include "engine/RunningPlan.h"

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BasicPlan.h"
#include "engine/BehaviourPool.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/RuleBook.h"
#include "engine/SimplePlanTree.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Configuration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Parameter.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/scheduler/Scheduler.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/common_defines.h>
#include <alica_common_config/debug_output.h>

#include <cstddef>
#include <iostream>

namespace alica
{

namespace
{
AlicaTime s_assignmentProtectionTime = AlicaTime::zero();
}

void RunningPlan::init(const YAML::Node& config)
{
    s_assignmentProtectionTime = AlicaTime::milliseconds(config["Alica"]["AssignmentProtectionTime"].as<unsigned long>());
}

void RunningPlan::setAssignmentProtectionTime(AlicaTime t)
{
    s_assignmentProtectionTime = t;
}

RunningPlan::RunningPlan(AlicaEngine* ae, const Configuration* configuration)
        : _ae(ae)
        , _planType(nullptr)
        , _behaviour(false)
        , _assignment()
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
        , _basicPlan(nullptr)
        , _parent(nullptr)
        , _configuration(configuration)
{
}

RunningPlan::~RunningPlan()
{
    if (_parent) {
        _parent->removeChild(this);
    }
}

RunningPlan::RunningPlan(AlicaEngine* ae, const Plan* plan, const Configuration* configuration)
        : _ae(ae)
        , _planType(nullptr)
        , _behaviour(false)
        , _assignment(plan)
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
        , _basicPlan(nullptr)
        , _parent(nullptr)
        , _configuration(configuration)
{
    _activeTriple.abstractPlan = plan;
}

RunningPlan::RunningPlan(AlicaEngine* ae, const PlanType* pt, const Configuration* configuration)
        : _ae(ae)
        , _planType(pt)
        , _behaviour(false)
        , _assignment()
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
        , _basicPlan(nullptr)
        , _parent(nullptr)
        , _configuration(configuration)
{
}

RunningPlan::RunningPlan(AlicaEngine* ae, const Behaviour* b, const Configuration* configuration)
        : _ae(ae)
        , _planType(nullptr)
        , _activeTriple(b, nullptr, nullptr)
        , _behaviour(true)
        , _assignment()
        , _basicBehaviour(nullptr)
        , _basicPlan(nullptr)
        , _cycleManagement(ae, this)
        , _parent(nullptr)
        , _configuration(configuration)
{
}

bool RunningPlan::isDeleteable() const
{
    if (!_children.empty()) {
        return false; // children deregister from their parents
    }
    if (_status.active == PlanActivity::InActive) {
        return true; // shortcut for plans from planselector
    }
    return isRetired() && (!isBehaviour() || !_ae->getBehaviourPool().isBehaviourRunningInContext(*this));
}

void RunningPlan::preTick()
{
    if (isRetired()) {
        return;
    }
    evalRuntimeCondition();
    for (RunningPlan* c : _children) {
        c->preTick();
    }
}
/**
 * Called once per Engine iteration, performs all neccessary checks and executes rules from the rulebook.
 * @param rules a RuleBook
 * @return PlanChange a PlanChange
 */
PlanChange RunningPlan::tick(RuleBook* rules)
{
    if (isRetired()) {
        return PlanChange::NoChange;
    }
    _cycleManagement.update();
    PlanChange myChange = rules->visit(*this);
    if (isRetired()) {
        return myChange;
    }
    PlanChange childChange = PlanChange::NoChange;
    // attention: do not use for each here: children are modified
    for (int i = 0; i < static_cast<int>(_children.size()); ++i) {
        RunningPlan* rp = _children[i];
        childChange = rules->updateChange(childChange, rp->tick(rules));
    }
    if (childChange != PlanChange::NoChange && childChange != PlanChange::InternalChange) {
        myChange = rules->updateChange(myChange, rules->visit(*this));
    }
    return myChange;
}

/**
 * Indicates whether an allocation is needed in the RunningPlan.ActiveState.
 * If set to true, the next engine iteration will perform a task allocation and set it to false.
 * true if allocation is needed, otherwise false
 */
void RunningPlan::setAllocationNeeded(bool need)
{
    _status.allocationNeeded = need;
}

/**
 * Evaluates the precondition of the associated plan.
 * @return Whether the precondition currently holds or not.
 */
bool RunningPlan::evalPreCondition() const
{
    if (_activeTriple.abstractPlan == nullptr) {
        ALICA_ERROR_MSG("Cannot Eval Condition, Plan is null");
        assert(false);
    }

    const PreCondition* preCondition = nullptr;
    if (const Behaviour* behaviour = dynamic_cast<const Behaviour*>(_activeTriple.abstractPlan)) {
        preCondition = behaviour->getPreCondition();
    }
    if (const Plan* plan = dynamic_cast<const Plan*>(_activeTriple.abstractPlan)) {
        preCondition = plan->getPreCondition();
    }
    if (preCondition == nullptr) {
        return true;
    }
    try {
        return preCondition->evaluate(*this, _ae->getWorldModel());
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("Exception in precondition: " << e.what());
        return false;
    }
}

/**
 * Evals the runtime condition of the associated plan.
 * @return Whether the runtime currently holds or not.
 */
bool RunningPlan::evalRuntimeCondition() const
{
    if (_activeTriple.abstractPlan == nullptr) {
        ALICA_ERROR_MSG("Cannot Eval Condition, Plan is null");
        throw std::exception();
    }
    const RuntimeCondition* runtimeCondition = nullptr;
    if (const Behaviour* behaviour = dynamic_cast<const Behaviour*>(_activeTriple.abstractPlan)) {
        runtimeCondition = behaviour->getRuntimeCondition();
    }
    if (const Plan* plan = dynamic_cast<const Plan*>(_activeTriple.abstractPlan)) {
        runtimeCondition = plan->getRuntimeCondition();
    }
    if (runtimeCondition == nullptr) {
        _status.runTimeConditionStatus = EvalStatus::True;
        return true;
    }
    try {
        bool ret = runtimeCondition->evaluate(*this, _ae->getWorldModel());
        _status.runTimeConditionStatus = (ret ? EvalStatus::True : EvalStatus::False);
        return ret;
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("Exception in runtimecondition: " << _activeTriple.abstractPlan->getName() << " " << e.what());
        _status.runTimeConditionStatus = EvalStatus::False;
        return false;
    }
}

void RunningPlan::addChildren(const std::vector<RunningPlan*>& runningPlans)
{
    _children.reserve(_children.size() + runningPlans.size());
    for (RunningPlan* r : runningPlans) {
        r->setParent(this);
        _children.push_back(r);
        auto iter = _failedSubPlans.find(r->getActivePlan());
        if (iter != _failedSubPlans.end()) {
            r->_status.failCount = iter->second;
        }
        if (isActive()) {
            r->activate();
        }
    }
}
void RunningPlan::removeChild(RunningPlan* rp)
{
    auto it = std::find(_children.begin(), _children.end(), rp);
    if (it != _children.end()) {
        (*it)->_parent = nullptr;
        _children.erase(it);
    }
}
/**
 * Move this very robot to another state. Performs all neccessary operations, such as updating the assignment.
 * @param nextState A State
 */
void RunningPlan::moveState(const State* nextState)
{
    deactivateChildren();
    clearChildren();
    _assignment.moveAllFromTo(_activeTriple.entryPoint, _activeTriple.state, nextState);
    useState(nextState);
    _failedSubPlans.clear();
}

/**
 * Simple method to recursively print the plan-tree.
 */
void RunningPlan::printRecursive() const
{
    for (const RunningPlan* c : _children) {
        c->printRecursive();
    }
    if (_children.empty()) {
        std::cout << "END CHILDREN of " << (_activeTriple.abstractPlan == nullptr ? "NULL" : _activeTriple.abstractPlan->getName()) << std::endl;
    }
}

void RunningPlan::usePlan(const AbstractPlan* plan)
{
    if (_activeTriple.abstractPlan != plan) {
        _status.planStartTime = _ae->getAlicaClock().now();
        revokeAllConstraints();
        _activeTriple.abstractPlan = plan;
        _status.runTimeConditionStatus = EvalStatus::Unknown;
    }
}

void RunningPlan::useEntryPoint(const EntryPoint* value)
{
    if (_activeTriple.entryPoint != value) {
        AgentId mid = getOwnID();
        _assignment.removeAgent(mid);
        _activeTriple.entryPoint = value;
        if (value != nullptr) {
            useState(value->getState());
            _assignment.addAgent(mid, _activeTriple.entryPoint, _activeTriple.state);
        }
    }
}

void RunningPlan::useState(const State* s)
{
    if (_activeTriple.state != s) {
        ALICA_ASSERT(s == nullptr || (_activeTriple.entryPoint && _activeTriple.entryPoint->isStateReachable(s)));
        _activeTriple.state = s;
        _status.stateStartTime = _ae->getAlicaClock().now();
        if (s != nullptr) {
            if (s->isFailureState()) {
                _status.status = PlanStatus::Failed;
            } else if (s->isSuccessState()) {
                AgentId mid = getOwnID();
                _assignment.editSuccessData(_activeTriple.entryPoint).push_back(mid);
                _ae->editTeamManager().setSuccess(mid, _activeTriple.abstractPlan, _activeTriple.entryPoint);
            }
        }
    }
}

/**
 * Gets the PlanStatus of the currently executed plan.
 */
PlanStatus RunningPlan::getStatus() const
{
    if (_basicBehaviour != nullptr) {
        if (_basicBehaviour->isSuccess()) {
            return PlanStatus::Success;
        } else if (_basicBehaviour->isFailure()) {
            return PlanStatus::Failed;
        } else {
            return PlanStatus::Running;
        }
    }
    if (_assignment.isSuccessful()) {
        return PlanStatus::Success;
    }
    return _status.status;
}

void RunningPlan::clearFailures()
{
    _status.failCount = 0;
}

/**
 * Clears the failure history of failed plans.
 */
void RunningPlan::clearFailedChildren()
{
    _failedSubPlans.clear();
}

void RunningPlan::addFailure()
{
    ++_status.failCount;
    setFailureHandlingNeeded(true);
}

/**
 * Returns the number of failures detected while this RunningPlan was executed.
 */
int RunningPlan::getFailureCount() const
{
    return _status.failCount;
}

void RunningPlan::deactivateChildren()
{
    for (RunningPlan* r : _children) {
        r->deactivate();
    }
}

/**
 * Remove all children without passing any command to them.
 */
void RunningPlan::clearChildren()
{
    for (RunningPlan* r : _children) {
        r->_parent = nullptr;
    }
    _children.clear();
}

/**
 * Adapt the assignment of this plan to the one supplied. This can also change plan
 * @param r A RunningPlan
 */
void RunningPlan::adaptAssignment(const RunningPlan& replacement)
{
    _assignment.adaptTaskChangesFrom(replacement.getAssignment());
    const State* newState = _assignment.getStateOfAgent(getOwnID());

    bool reactivate = false;

    if (_activeTriple.state != newState) {
        _status.active = PlanActivity::InActive;
        deactivateChildren();
        revokeAllConstraints();
        clearChildren();
        addChildren(replacement.getChildren());
        reactivate = true;
    } else {
        AgentGrp robotsJoined;
        _assignment.getAgentsInState(newState, robotsJoined);
        for (RunningPlan* c : _children) {
            c->limitToRobots(robotsJoined);
        }
    }

    usePlan(replacement.getActivePlan());
    _activeTriple.entryPoint = replacement.getActiveEntryPoint();
    useState(newState);
    if (reactivate) {
        activate();
    }
}

/**
 * Indicate that an AbstractPlan has failed while being a child of this plan.
 * @param child a AbstractPlan
 */
void RunningPlan::setFailedChild(const AbstractPlan* child)
{
    if (_failedSubPlans.find(child) != _failedSubPlans.end()) {
        _failedSubPlans.at(child)++;
    } else {
        _failedSubPlans.insert(std::pair<const AbstractPlan*, int>(child, 1));
    }
}

void RunningPlan::setFailureHandlingNeeded(bool failHandlingNeeded)
{
    if (failHandlingNeeded) {
        _status.status = PlanStatus::Failed;
    } else {
        if (_status.status == PlanStatus::Failed) {
            _status.status = PlanStatus::Running;
        }
    }
    _status.failHandlingNeeded = failHandlingNeeded;
}

/**
 *  General Visitor pattern for the plan graph.
 *  @param vis A IPlanTreeVisitor
 */
void RunningPlan::accept(IPlanTreeVisitor* vis)
{
    vis->visit(*this);

    for (RunningPlan* child : _children) {
        assert(!child->isRetired());
        if (!child->isRetired()) {
            child->accept(vis);
        }
    }
}

/**
 *  Deactivate this plan, to be called before the plan is removed from the graph.
 * Ensures that all sub-behaviours are stopped and all constraints are revoked.
 */
void RunningPlan::deactivate()
{
    _status.active = PlanActivity::Retired;
    revokeAllConstraints();
    deactivateChildren();

    if (isBehaviour()) {
        _ae->editBehaviourPool().stopBehaviour(*this);
    } else {
        _ae->getTeamObserver().notifyRobotLeftPlan(_activeTriple.abstractPlan);
        _ae->editPlanPool().stopPlan(*this);
    }
}

/**
 * Tests whether any child has a specific status.
 * @param A PlanStatus
 * @return bool
 */
bool RunningPlan::isAnyChildStatus(PlanStatus ps) const
{
    for (const RunningPlan* child : _children) {
        assert(!child->isRetired());
        if (ps == child->getStatus()) {
            return true;
        }
    }
    return false;
}

/**
 * Tests whether all children have the specific status.
 * @param A PlanStatus
 * @returns bool
 */
bool RunningPlan::areAllChildrenStatus(PlanStatus ps) const
{
    for (const RunningPlan* child : _children) {
        assert(!child->isRetired());
        if (ps != child->getStatus()) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (_activeTriple.state) {
        return _children.size() >= _activeTriple.state->getConfAbstractPlanWrappers().size();
    }
    return true;
}

/**
 * Tests whether any child has a task with the given status.
 * @param A TaskStatus
 * @return bool
 */
bool RunningPlan::isAnyChildTaskSuccessful() const
{
    for (const RunningPlan* child : _children) {
        if (child->isBehaviour()) {
            // Behaviours have no task status!
            continue;
        }
        if (child->getAssignment().isAnyTaskSuccessful()) {
            return true;
        }
    }
    return false;
}

bool RunningPlan::amISuccessful() const
{
    if (isBehaviour()) { // behaviors only have a simple success flag
        return getStatus() == PlanStatus::Success;
    }
    return getAssignment().isAgentSuccessful(_ae->getTeamManager().getLocalAgentID(), _activeTriple.entryPoint);
}

bool RunningPlan::amISuccessfulInAnyChild() const
{
    for (const RunningPlan* child : _children) {
        if (child->amISuccessful()) {
            return true;
        }
    }
    return false;
}

/**
 * Activate this plan, called when it is inserted into the plan graph.
 */
void RunningPlan::activate()
{
    assert(_status.active != PlanActivity::Retired);
    _status.active = PlanActivity::Active;
    if (isBehaviour()) {
        _ae->editBehaviourPool().startBehaviour(*this);
    } else if (_activeTriple.abstractPlan) {
        _ae->editPlanPool().startPlan(*this);
    }

    attachPlanConstraints();
    for (RunningPlan* r : _children) {
        r->activate();
    }
}

/**
 * Removes any robot not in robots
 * @param robots The set of robots that can participate in this running plan.
 */
void RunningPlan::limitToRobots(const AgentGrp& robots)
{
    if (isBehaviour()) {
        return;
    }
    if (!_cycleManagement.mayDoUtilityCheck()) {
        return;
    }

    const bool ownStateWasTouched = _assignment.removeAllNotIn(robots, _activeTriple.state);

    if (ownStateWasTouched) {
        for (RunningPlan* c : _children) {
            c->limitToRobots(robots);
        }
    }
}

/**
 * //convenience method as recursive case might have been called for the children already
 */
void RunningPlan::revokeAllConstraints()
{
    _constraintStore.clear();
}

void RunningPlan::attachPlanConstraints()
{
    if (const Behaviour* behaviour = dynamic_cast<const Behaviour*>(_activeTriple.abstractPlan)) {
        _constraintStore.addCondition(behaviour->getPreCondition());
        _constraintStore.addCondition(behaviour->getRuntimeCondition());
    } else if (const Plan* plan = dynamic_cast<const Plan*>(_activeTriple.abstractPlan)) {
        _constraintStore.addCondition(plan->getPreCondition());
        _constraintStore.addCondition(plan->getRuntimeCondition());
    }
}

bool RunningPlan::recursiveUpdateAssignment(const std::vector<const SimplePlanTree*>& spts, AgentGrp& availableAgents, const AgentGrp& noUpdates, AlicaTime now)
{
    if (isBehaviour()) {
        return false;
    }
    const bool keepTask = _status.planStartTime + s_assignmentProtectionTime > now;
    const bool keepState = _status.stateStartTime + s_assignmentProtectionTime > now;
    const bool auth = _cycleManagement.haveAuthority();

    // if keepTask, the task Assignment should not be changed!
    bool ret = false;
    AllocationDifference& aldif = _cycleManagement.editNextDifference();
    for (const SimplePlanTree* spt : spts) {
        AgentId id = spt->getAgentId();
        const bool freezeAgent = keepState && _assignment.getStateOfAgent(id) == getActiveState();
        if (freezeAgent) {
            continue;
        }
        if (spt->getState()->getInPlan() != _activeTriple.abstractPlan) { // the robot is no longer participating in this plan
            if (!keepTask && !auth) {
                const EntryPoint* ep = _assignment.getEntryPointOfAgent(id);
                if (ep != nullptr) {
                    _assignment.removeAgentFrom(id, ep);
                    ret = true;
                    aldif.editSubtractions().emplace_back(ep, id);
                }
            }
        } else {
            if (keepTask || auth) { // Update only state, and that only if it is in the reachability graph of its
                                    // current entrypoint, else
                // ignore
                const EntryPoint* cep = _assignment.getEntryPointOfAgent(id);
                if (cep != nullptr) {
                    if (cep->isStateReachable(spt->getState())) {
                        _assignment.setState(id, spt->getState(), cep);
                    }
                } else { // robot was not expected to be here during protected assignment time, add it.
                    _assignment.addAgent(id, spt->getEntryPoint(), spt->getState());
                    aldif.editAdditions().emplace_back(spt->getEntryPoint(), id);
                }
            } else { // Normal Update
                const EntryPoint* ep = _assignment.getEntryPointOfAgent(id);
                ret |= _assignment.updateAgent(id, spt->getEntryPoint(), spt->getState());
                if (spt->getEntryPoint() != ep) {
                    aldif.editAdditions().emplace_back(spt->getEntryPoint(), id);
                    if (ep != nullptr) {
                        aldif.editSubtractions().emplace_back(ep, id);
                    }
                }
            }
        }
    }

    AgentGrp rem;
    if (!keepTask) { // remove any robot no longer available in the spts (auth flag obey here, as robot might be
                     // unavailable)
        // EntryPoint[] eps = this.Assignment.GetEntryPoints();
        AgentId ownId = getOwnID();
        for (int i = 0; i < _assignment.getEntryPointCount(); ++i) {
            const EntryPoint* ep = _assignment.getEntryPoint(i);
            rem.clear();
            AssignmentView robs = _assignment.getAgentsWorking(i);
            for (AgentId rob : robs) {
                if (rob == ownId) {
                    continue;
                }
                const bool freezeAgent = keepState && _assignment.getStateOfAgent(rob) == getActiveState();
                if (freezeAgent) {
                    continue;
                }
                bool found = false;
                if (std::find(noUpdates.begin(), noUpdates.end(), rob) != noUpdates.end()) {
                    // found = true;
                    continue;
                }
                for (const SimplePlanTree* spt : spts) {
                    if (spt->getAgentId() == rob) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    rem.push_back(rob);
                    aldif.editSubtractions().emplace_back(ep, rob);
                    ret = true;
                }
            }
            _assignment.removeAllFrom(rem, ep);
        }
    }

    // enforce consistency between RA and PlanTree by removing robots deemed inactive:
    if (!auth) { // under authority do not remove robots from assignment
        for (int i = 0; i < _assignment.getEntryPointCount(); ++i) {
            const EntryPoint* ep = _assignment.getEntryPoint(i);
            rem.clear();
            AssignmentView robs = _assignment.getAgentsWorking(i);
            for (AgentId rob : robs) {
                if (std::find(availableAgents.begin(), availableAgents.end(), rob) == availableAgents.end()) {
                    rem.push_back(rob);
                    aldif.editSubtractions().emplace_back(ep, rob);
                    ret = true;
                }
            }
            _assignment.removeAllFrom(rem, ep);
        }
    }

    aldif.setReason(AllocationDifference::Reason::message);

    // Update Success Collection:
    _ae->editTeamObserver().updateSuccessCollection(static_cast<const Plan*>(getActivePlan()), _assignment.editSuccessData());

    // If Assignment Protection Time for newly started plans is over, limit available robots to those in this active
    // state.
    if (!auth) {
        AgentsInStateView agentsJoined = _assignment.getAgentsInState(getActiveState());
        for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
            if (std::find(agentsJoined.begin(), agentsJoined.end(), *iter) == agentsJoined.end()) {
                iter = availableAgents.erase(iter);
            } else {
                ++iter;
            }
        }
    } else { // in case of authority, remove all that are not assigned to same task
        AssignmentView agentsJoined = _assignment.getAgentsWorking(getActiveEntryPoint());
        for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
            if (std::find(agentsJoined.begin(), agentsJoined.end(), *iter) == agentsJoined.end()) {
                iter = availableAgents.erase(iter);
            } else {
                ++iter;
            }
        }
    }
    // Give Plans to children
    for (RunningPlan* r : _children) {
        if (r->isBehaviour()) {
            continue;
        }
        std::vector<const SimplePlanTree*> newcspts;
        for (const SimplePlanTree* spt : spts) {
            if (spt->getState() == _activeTriple.state) {
                for (const std::unique_ptr<SimplePlanTree>& cspt : spt->getChildren()) {
                    if (cspt->getState()->getInPlan() == r->getActivePlan()) {
                        newcspts.push_back(cspt.get());
                        break;
                    }
                }
            }
        }
        ret |= r->recursiveUpdateAssignment(newcspts, availableAgents, noUpdates, now);
    }
    return ret;
}

void RunningPlan::toMessage(IdGrp& message, const RunningPlan*& o_deepestNode, int& o_depth, int curDepth) const
{
    if (isBehaviour() || isRetired()) {
        return;
    }
    if (_activeTriple.state != nullptr) {
        message.push_back(_activeTriple.state->getId());
    } else {
        return;
    }
    if (curDepth > o_depth) {
        o_depth = curDepth;
        o_deepestNode = this;
    }
    if (_children.size() > 0) {
        message.push_back(-1);
        for (const RunningPlan* r : _children) {
            r->toMessage(message, o_deepestNode, o_depth, curDepth + 1);
        }
        message.push_back(-2);
    }
}

    AgentId RunningPlan::getOwnID() const
{
    return _ae->getTeamManager().getLocalAgentID();
}

/**
 * Tries to find a given key in the configuration of this RunningPlan and
 * writes the corresponding value into valueOut.
 * @param key The key to be found in the configuration.
 * @param valueOut The value, corresponding to the given key.
 * @return True, if the configuration and the key was present. False, otherwise.
 */
bool RunningPlan::getParameter(const std::string& key, std::string& valueOut) const
{
    if (!_configuration) {
        return false;
    }

    const auto& parameter = _configuration->getParameters().find(key);
    if (parameter != _configuration->getParameters().end()) {
        valueOut = parameter->second->getValue();
        return true;
    } else {
        return false;
    }
}

const Configuration* RunningPlan::getConfiguration() const
{
    return _configuration;
}

std::ostream& operator<<(std::ostream& out, const RunningPlan& r)
{
    out << "######## RP ##########" << std::endl;
    PlanStateTriple ptz = r.getActiveTriple();
    out << "Plan: " + (ptz.abstractPlan != nullptr ? ptz.abstractPlan->getName() : "NULL") << std::endl;
    out << "PlanType: " << (r.getPlanType() != nullptr ? r.getPlanType()->getName() : "NULL") << std::endl;
    out << "ActState: " << (ptz.state != nullptr ? ptz.state->getName() : "NULL") << std::endl;
    out << "Task: " << (ptz.entryPoint != nullptr ? ptz.entryPoint->getTask()->getName() : "NULL") << std::endl;
    out << "IsBehaviour: " << r.isBehaviour() << "\t";
    if (r.isBehaviour()) {
        out << "Behaviour: " << (r.getBasicBehaviour() == nullptr ? "NULL" : r.getBasicBehaviour()->getName()) << std::endl;
    }
    const RunningPlan::PlanStatusInfo& psi = r.getStatusInfo();
    out << "AllocNeeded: " << psi.allocationNeeded << std::endl;
    out << "FailHandlingNeeded: " << psi.failHandlingNeeded << "\t";
    out << "FailCount: " << psi.failCount << std::endl;
    out << "Activity: " << getPlanActivityName(psi.active) << std::endl;
    if (!r.isBehaviour()) {
        out << "Status: " << getPlanStatusName(psi.status) << std::endl;
    } else if (r.getBasicBehaviour()->isSuccess()) {
        out << "Status: " << getPlanStatusName(PlanStatus::Success) << std::endl;
    } else if (r.getBasicBehaviour()->isFailure()) {
        out << "Status: " << getPlanStatusName(PlanStatus::Failed) << std::endl;
    } else {
        out << "Status: " << getPlanStatusName(PlanStatus::Running) << std::endl;
    }
    out << std::endl;
    if (!r.isBehaviour()) {
        out << "Assignment:" << r._assignment;
    }
    out << "Children: " << r._children.size();
    if (!r._children.empty()) {
        out << " ( ";
        for (const RunningPlan* c : r._children) {
            if (c->_activeTriple.abstractPlan == nullptr) {
                out << "NULL PLAN, ";
            } else
                out << c->_activeTriple.abstractPlan->getName() + ", ";
        }
        out << ")";
    }
    out << std::endl << "CycleManagement - Assignment Overridden: " << (r._cycleManagement.isOverridden() ? "true" : "false") << std::endl;
    out << std::endl << "########## ENDRP ###########" << std::endl;
    return out;
}

} /* namespace alica */
