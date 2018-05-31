#include "engine/RunningPlan.h"

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/RuleBook.h"
#include "engine/SimplePlanTree.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/StateCollection.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

#include <alica_common_config/common_defines.h>

#include <iostream>

using std::shared_ptr;

namespace alica
{

AlicaTime RunningPlan::assignmentProtectionTime = AlicaTime::zero();

void RunningPlan::init()
{
    assignmentProtectionTime =
            AlicaTime::milliseconds((*supplementary::SystemConfig::getInstance())["Alica"]->get<unsigned long>("Alica.AssignmentProtectionTime", NULL));
}

RunningPlan::RunningPlan(AlicaEngine* ae)
        : _ae(ae)
        , _planType(nullptr)
        , _behaviour(false)
        , _assignment()
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
{
}

RunningPlan::~RunningPlan() {}

RunningPlan::RunningPlan(AlicaEngine* ae, const Plan* plan)
        : _ae(ae)
        , _planType(nullptr)
        , _behaviour(false)
        , _assignment(plan)
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
{
    _activeTriple._plan = plan;
}

RunningPlan::RunningPlan(AlicaEngine* ae, const PlanType* pt)
        : _ae(ae)
        , _planType(pt)
        , _behaviour(false)
        , _assignment()
        , _cycleManagement(ae, this)
        , _basicBehaviour(nullptr)
{
}

RunningPlan::RunningPlan(AlicaEngine* ae, const BehaviourConfiguration* bc)
        : _ae(ae)
        , _planType(nullptr)
        , _activeTriple(bc, nullptr, nullptr)
        , _behaviour(true)
        , _assignment()
        , _basicBehaviour(nullptr)
        , _cycleManagement(ae, this)
{
}

/*
bool RunningPlan::getFailHandlingNeeded() const
{
    return _failHandlingNeeded;
}
void RunningPlan::setFailHandlingNeeded(bool failHandlingNeeded)
{
    if (failHandlingNeeded) {
        _status = PlanStatus::Failed;
    } else {
        if (_status == PlanStatus::Failed) {
            _status = PlanStatus::Running;
        }
    }
    _failHandlingNeeded = failHandlingNeeded;
}
*/

/**
 * Called once per Engine iteration, performs all neccessary checks and executes rules from the rulebook.
 * @param rules a RuleBook
 * @return PlanChange a PlanChange
 */
PlanChange RunningPlan::tick(RuleBook* rules)
{
    _cycleManagement->update();
    PlanChange myChange = rules->visit(shared_from_this());
    PlanChange childChange = PlanChange::NoChange;
    // attention: do not use for each here: children are modified
    for (int i = 0; i < static_cast<int>(_children.size()); i++) {
        auto it = _children.begin();
        advance(it, i);
        shared_ptr<RunningPlan> rp = *it;
        childChange = rules->updateChange(childChange, rp->tick(rules));
    }
    if (childChange != PlanChange::NoChange && childChange != PlanChange::InternalChange) {
        myChange = rules->updateChange(myChange, rules->visit(shared_from_this()));
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
    _allocationNeeded = need;
}

/**
 * Evaluates the precondition of the associated plan.
 * @return Whether the precondition currently holds or not.
 */
bool RunningPlan::evalPreCondition()
{
    if (_plan == nullptr) {
        std::cerr << "Cannot Eval Condition, Plan is null" << std::endl;
        throw std::exception();
    }
    if (_plan->getPreCondition() == nullptr) {
        return true;
    }
    try {
        return _plan->getPreCondition()->evaluate(shared_from_this());
    } catch (std::exception& e) {
        std::cerr << "Exception in precondition: " << e.what() << std::endl;
        return false;
    }
}

/**
 * Evals the runtime condition of the associated plan.
 * @return Whether the runtime currently holds or not.
 */
bool RunningPlan::evalRuntimeCondition()
{
    if (_plan == nullptr) {
        std::cerr << "Cannot Eval Condition, Plan is null" << std::endl;
        throw std::exception();
    }
    if (_plan->getRuntimeCondition() == nullptr) {
        return true;
    }
    try {
        return _plan->getRuntimeCondition()->evaluate(shared_from_this());
    } catch (std::exception& e) {
        std::cerr << "Exception in runtimecondition: " << _plan->getName() << e.what() << std::endl;
        return false;
    }
}

void RunningPlan::addChildren(const std::vector<RunningPlan*>& runningPlans)
{
    _children.reserve(_children.size() + runningPlans.size());
    for (RunningPlan* r : runningPlans) {
        r->setParent(this);
        _children.push_back(r);
        auto iter = _failedSubPlans.find(r->getPlan());
        if (iter != _failedSubPlans.end()) {
            r->_failCount = iter->second;
        }
        if (_active) {
            r->activate();
        }
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
    _assignment->moveRobots(_activeState, nextState);
    setActiveState(nextState);
    _failedSubPlans.clear();
}

/*
void RunningPlan::setChildren(list<shared_ptr<RunningPlan>> children)
{
    _children = children;
}
*/

/*void RunningPlan::setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour)
{
    _basicBehaviour = basicBehaviour;
}
*/
/**
 * Simple method to recursively print the plan-tree.
 */
void RunningPlan::printRecursive() const
{
    for (const RunningPlan* c : _children) {
        c->printRecursive();
    }
    if (_children.empty()) {
        std::cout << "END CHILDREN of " << (_activeTriple.plan == nullptr ? "NULL" : _activeTriple.plan->getName()) << std::endl;
    }
}

/*
void RunningPlan::setActive(bool active)
{
    _active = active;
}
*/
void RunningPlan::usePlan(const AbstractPlan* plan)
{
    if (_planStateTriple.plan != plan) {
        _planStartTime = _ae->getAlicaClock()->now();
        revokeAllConstraints();
        _planStateTriple.plan = plan;
    }
}

void RunningPlan::useEntryPoint(const EntryPoint* value)
{
    if (_activeTriple.entryPoint != value) {
        AgentIdConstPtr mid = getOwnID();
        _assignment.removeRobot(mid);
        _activeTriple.entryPoint = value;
        if (value != nullptr) {
            setOwnActiveState(value->getState());
            _assignment.addRobot(mid, _activeTriple.entryPoint, _activeTriple.state);
        }
    }
}

void RunningPlan::useState(const State* s)
{
    if (_activeTriple.state != s) {
        ALICA_ASSERT(s == null || (_activeTriple.entryPoint && _activeTriple.entryPoint->isStateReachable(s)));
        _activeTriple.state = s;
        _status.stateStartTime = _ae->getAlicaClock()->now();
        if (s != nullptr) {
            if (s->isFailureState()) {
                _status.status = PlanStatus::Failed;
            } else if (s->isSuccessState()) {
                AgentIdConstPtr mid = getOwnID();
                _assignment.getEpSuccessMapping().getRobots(_activeTriple.entryPoint)->push_back(mid);
                _ae->getTeamManager()->setSuccess(mid, _activeTriple.plan, _activeTriple.entryPoint);
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
    if (_assignment.isSuccessfull()) {
        return PlanStatus::Success;
    }
    return _status.status;
}

void RunningPlan::clearFailures()
{
    _failCount = 0;
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
    _failCount++;
    _failHandlingNeeded = true;
}

/**
 * Returns the number of failures detected while this RunningPlan was executed.
 */
int RunningPlan::getFailure()
{
    return _failCount;
}

void RunningPlan::deactivateChildren()
{
    for (shared_ptr<RunningPlan>& r : _children) {
        r->deactivate();
    }
}

/**
 * Remove all children without passing any command to them.
 */
void RunningPlan::clearChildren()
{
    _children.clear();
}

/**
 * Adapt the assignment of this plan to the one supplied. This can also change plan
 * @param r A RunningPlan
 */
void RunningPlan::adaptAssignment(shared_ptr<RunningPlan> r)
{
    const State* newState = r->getAssignment()->getRobotStateMapping()->getStateOfRobot(getOwnID());
    r->getAssignment()->getRobotStateMapping()->reconsiderOldAssignment(_assignment, r->getAssignment());
    bool reactivate = false;

    if (_activeState != newState) {
        _active = false;
        deactivateChildren();
        revokeAllConstraints();
        clearChildren();
        addChildren(*r->getChildren());
        reactivate = true;
    } else {
        AgentGrp robotsJoined;
        r->getAssignment()->getRobotStateMapping()->getRobotsInState(newState, robotsJoined);
        for (shared_ptr<RunningPlan>& r : _children) {
            r->limitToRobots(robotsJoined);
        }
    }

    _plan = r->getPlan();
    _activeEntryPoint = r->getOwnEntryPoint();
    _assignment = r->_assignment;
    setActiveState(newState);
    if (reactivate) {
        activate();
    }
}

/**
 * Indicates whether this running plan represents a behaviour.
 */
/*std::shared_ptr<CycleManager> RunningPlan::getCycleManagement()
{
    return _cycleManagement;
}*/

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

/**
 *  General Visitor pattern for the plan graph.
 *  @param vis A IPlanTreeVisitor
 */
void RunningPlan::accept(IPlanTreeVisitor* vis)
{
    vis->visit(*this);

    for (RunningPlan* child : _children) {
        child->accept(vis);
    }
}

/**
 *  Deactivate this plan, to be called before the plan is removed from the graph.
 * Ensures that all sub-behaviours are stopped and all constraints are revoked.
 */
void RunningPlan::deactivate()
{
    _active = false;
    if (isBehaviour()) {
        _ae->getBehaviourPool()->stopBehaviour(shared_from_this());
    } else {
        _ae->getTeamObserver()->notifyRobotLeftPlan(_plan);
    }
    revokeAllConstraints();
    deactivateChildren();
}

/**
 * Tests whether all children have the specific status.
 * @param A PlanStatus
 * @returns bool
 */
bool RunningPlan::allChildrenStatus(PlanStatus ps)
{
    for (const std::shared_ptr<RunningPlan>& child : _children) {
        if (ps != child->getStatus()) {
            return false;
        }
    }
    return true;
}

/**
 * Tests whether any child has a task with the given status.
 * @param A TaskStatus
 * @return bool
 */
bool RunningPlan::anyChildrenTaskSuccess()
{
    for (auto child : _children) {
        if (child->isBehaviour()) {
            // Behaviours have no task status!
            continue;
        }

        auto childAssignment = child->getAssignment();
        if (!childAssignment) {
            // no assignment, so no successful task
            continue;
        }

        for (int i = 0; i < childAssignment->getEpSuccessMapping()->getCount(); i++) {
            // at least one robot must be successful && at least as much as minCard robots must be successful
            if (childAssignment->getEpSuccessMapping()->getRobots()[i]->size() > 0 &&
                    static_cast<int>(childAssignment->getEpSuccessMapping()->getRobots()[i]->size()) >=
                            childAssignment->getEpSuccessMapping()->getEntryPoints()[i]->getMinCardinality()) {
                return true;
            }
        }
    }

    return false;
}

/**
 * Activate this plan, called when it is inserted into the plan graph.
 */
void RunningPlan::activate()
{
    _active = true;
    if (isBehaviour()) {
        _ae->getBehaviourPool()->startBehaviour(shared_from_this());
    }
    attachPlanConstraints();
    for (shared_ptr<RunningPlan>& r : _children) {
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
    if (!_cycleManagement->mayDoUtilityCheck()) {
        return;
    }
    bool recurse = false;

    // TODO: move this code into assignment to avoid the call to getAllRobots.
    AgentGrp curRobots;
    _assignment->getAllRobots(curRobots);
    for (const supplementary::AgentID* r : curRobots) {
        if (find_if(robots.begin(), robots.end(), [&r](const supplementary::AgentID* id) { return *r == *id; }) == robots.end()) {
            if (_activeState != nullptr && _assignment->getRobotStateMapping()->getStateOfRobot(r) == _activeState) {
                recurse = true;
            }
            _assignment->removeRobot(r);
        }
    }
    if (recurse) {
        for (shared_ptr<RunningPlan>& c : _children) {
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
    _constraintStore.addCondition(_plan->getPreCondition());
    _constraintStore.addCondition(_plan->getRuntimeCondition());
}

bool RunningPlan::recursiveUpdateAssignment(
        list<shared_ptr<SimplePlanTree>> spts, AgentGrp& availableAgents, list<const supplementary::AgentID*> noUpdates, AlicaTime now)
{
    if (isBehaviour()) {
        return false;
    }
    bool keepTask = ((_planStartTime + assignmentProtectionTime) > now);
    bool auth = _cycleManagement->haveAuthority();

    // if keepTask, the task Assignment should not be changed!
    bool ret = false;
    AllocationDifference* aldif = new AllocationDifference();
    for (shared_ptr<SimplePlanTree> spt : spts) {
        if (spt->getState()->getInPlan() != _plan) { // the robot is no longer participating in this plan
            if (!keepTask & !auth) {
                const EntryPoint* ep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
                if (ep != nullptr) {
                    getAssignment()->removeRobot(spt->getRobotId());
                    ret = true;
                    aldif->editSubtractions().emplace_back(ep, spt->getRobotId());
                }
            }
        } else {
            if (keepTask || auth) { // Update only state, and that only if it is in the reachability graph of its
                                    // current entrypoint, else
                // ignore
                const EntryPoint* cep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
                if (cep != nullptr) {
                    if (cep->isStateReachable(spt->getState())) {
                        getAssignment()->getRobotStateMapping()->setState(spt->getRobotId(), spt->getState());
                    }
                } else { // robot was not expected to be here during protected assignment time, add it.
                    getAssignment()->addRobot(spt->getRobotId(), spt->getEntryPoint(), spt->getState());
                    aldif->editAdditions().emplace_back(spt->getEntryPoint(), spt->getRobotId());
                }
            } else { // Normal Update
                const EntryPoint* ep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
                ret |= getAssignment()->updateRobot(spt->getRobotId(), spt->getEntryPoint(), spt->getState());
                if (spt->getEntryPoint() != ep) {
                    aldif->editAdditions().emplace_back(spt->getEntryPoint(), spt->getRobotId());
                    if (ep != nullptr) {
                        aldif->editSubtractions().emplace_back(ep, spt->getRobotId());
                    }
                }
            }
        }
    }

    list<const supplementary::AgentID*> rem;
    if (!keepTask) { // remove any robot no longer available in the spts (auth flag obey here, as robot might be
                     // unavailable)
        // EntryPoint[] eps = this.Assignment.GetEntryPoints();
        const supplementary::AgentID* ownId = getOwnID();
        for (int i = 0; i < getAssignment()->getEntryPointCount(); i++) {
            const EntryPoint* ep = getAssignment()->getEpRobotsMapping()->getEp(i);
            rem.clear();
            auto robs = getAssignment()->getRobotsWorking(ep);
            for (auto& rob : (*robs)) {
                if (*rob == *ownId)
                    continue;
                bool found = false;
                if (find_if(noUpdates.begin(), noUpdates.end(), [&rob](const supplementary::AgentID* id) { return *rob == *id; }) != noUpdates.end()) {
                    // found = true;
                    continue;
                }
                for (shared_ptr<SimplePlanTree> spt : spts) {
                    if (*(spt->getRobotId()) == *rob) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    rem.push_back(rob);
                    // this.Assignment.RemoveRobot(rob);
                    aldif->editSubtractions().emplace_back(ep, rob);
                    ret = true;
                }
            }
            for (auto& rob : rem) {
                getAssignment()->removeRobot(rob, ep);
            }
        }
    }

    // enforce consistency between RA and PlanTree by removing robots deemed inactive:
    if (!auth) { // under authority do not remove robots from assignment
        for (int i = 0; i < getAssignment()->getEntryPointCount(); i++) {
            const EntryPoint* ep = getAssignment()->getEpRobotsMapping()->getEp(i);
            rem.clear();
            auto robs = getAssignment()->getRobotsWorking(ep);
            for (auto& rob : (*robs)) {
                if (find_if(availableAgents.begin(), availableAgents.end(), [&rob](const supplementary::AgentID* id) { return *rob == *id; }) ==
                        availableAgents.end()) {
                    rem.push_back(rob);
                    aldif->editSubtractions().emplace_back(ep, rob);
                    ret = true;
                }
            }

            for (auto& rob : rem) {
                getAssignment()->removeRobot(rob, ep);
            }
        }
    }

    aldif->setReason(AllocationDifference::Reason::message);
    _cycleManagement->setNewAllocDiff(aldif);
    // Update Success Collection:
    _ae->getTeamObserver()->updateSuccessCollection((Plan*) getPlan(), getAssignment()->getEpSuccessMapping());

    // If Assignment Protection Time for newly started plans is over, limit available robots to those in this active
    // state.
    if (_stateStartTime + assignmentProtectionTime > now) {
        AgentGrp robotsJoined;
        getAssignment()->getRobotStateMapping()->getRobotsInState(getActiveState(), robotsJoined);
        for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
            if (std::find_if(robotsJoined.begin(), robotsJoined.end(), [&iter](const supplementary::AgentID* id) { return *(*iter) == *id; }) ==
                    robotsJoined.end()) {
                iter = availableAgents.erase(iter);
            } else {
                ++iter;
            }
        }
    } else if (auth) { // in case of authority, remove all that are not assigned to same task
        const AgentGrp* robotsJoined = getAssignment()->getRobotsWorking(getOwnEntryPoint());

        if (robotsJoined) {
            for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
                if (find_if(robotsJoined->begin(), robotsJoined->end(), [&iter](const supplementary::AgentID* id) { return *(*iter) == *id; }) ==
                        robotsJoined->end()) {
                    iter = availableAgents.erase(iter);
                } else {
                    ++iter;
                }
            }
        }
    }
    // Give Plans to children
    for (shared_ptr<RunningPlan>& r : _children) {
        if (r->isBehaviour()) {
            continue;
        }
        list<shared_ptr<SimplePlanTree>> newcspts;
        for (shared_ptr<SimplePlanTree> spt : spts) {
            if (spt->getState() == _activeState) {
                for (shared_ptr<SimplePlanTree> cspt : spt->getChildren()) {
                    if (cspt->getState()->getInPlan() == r->getPlan()) {
                        newcspts.push_back(cspt);
                        break;
                    }
                }
            }
        }
        ret |= r->recursiveUpdateAssignment(newcspts, availableAgents, noUpdates, now);
    }
    return ret;
}

void RunningPlan::toMessage(IdGrp& message, shared_ptr<const RunningPlan>& deepestNode, int& depth, int curDepth) const
{
    if (isBehaviour()) {
        return;
    }
    if (_activeState != nullptr) {
        message.push_back(_activeState->getId());
    } else {
        return;
    }
    if (curDepth > depth) {
        depth = curDepth;
        deepestNode = shared_from_this();
    }
    if (_children.size() > 0) {
        message.push_back(-1);
        for (const shared_ptr<RunningPlan>& r : _children) {
            r->toMessage(message, deepestNode, depth, curDepth + 1);
        }
        message.push_back(-2);
    }
}

/**
 * Tests whether any child has a specific status.
 * @param A PlanStatus
 * @return bool
 */
bool RunningPlan::anyChildrenStatus(PlanStatus ps)
{
    for (std::shared_ptr<RunningPlan>& child : _children) {
        if (ps == child->getStatus()) {
            return true;
        }
    }
    return false;
}

void RunningPlan::sendLogMessage(int level, const std::string& message) const
{
    _ae->getCommunicator()->sendLogMessage(level, message);
}

std::string RunningPlan::toString() const
{
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

std::ostream& operator<<(std::ostream& out, const RunningPlan& r)
{
    out << "######## RP ##########" << std::endl;
    PlanStateTriple ptz = r.getActiveTriple();
    out << "Plan: " + (ptz.plan != nullptr ? ptz.plan->getName() : "NULL") << std::endl;
    out << "PlanType: " << (r.getPlanType() != nullptr ? r.getPlanType()->getName() : "NULL") << std::endl;
    out << "ActState: " << (ptz.state != nullptr ? ptz.state->getName() : "NULL") << std::endl;
    out << "Task: " << (ptz.entryPoint != nullptr ? ptz.entryPoint->getTask()->getName() : "NULL") << std::endl;
    out << "IsBehaviour: " << r.isBehaviour() << "\t";
    if (r.isBehaviour()) {
        out << "Behaviour: " << (getBasicBehaviour() == nullptr ? "NULL" : r.getBasicBehaviour()->getName()) << std::endl;
    }
    const RunningPlan::PlanStatusInfo& psi = r.getStatusInfo();
    out << "AllocNeeded: " << psi.allocationNeeded << std::endl;
    out << "FailHandlingNeeded: " << psi.failHandlingNeeded << "\t";
    out << "FailCount: " << psi.failCount << std::endl;
    out << "IsActive: " << psi.active << std::endl;
    out << "Status: " << getPlanStatusName(psi.status) << std::endl;
    out << "AvailRobots: ";
    for (AgentIdConstPtr r : _robotsAvail) {
        out << " " << *r;
    }
    out << std::endl;
    if (!isBehaviour()) {
        out << "Assignment:" << _assignment;
    }
    out << "Children: " << _children.size();
    if (!_children.empty()) {
        out << " ( ";
        for (const RunningPlan* r : _children) {
            if (r->_activeTriple.plan == nullptr) {
                out << "NULL PLAN, ";
            } else
                out << r->_activeTriple.plan->getName() + ", ";
        }
        out << ")";
    }
    out << std::endl << "CycleManagement - Assignment Overridden: " << (_cycleManagement.isOverridden() ? "true" : "false") << std::endl;
    out << std::endl << "########## ENDRP ###########" << std::endl;
    return out;
}

} /* namespace alica */
