#include "engine/RunningPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaClock.h"
#include "engine/IAlicaCommunication.h"
#include "engine/BehaviourPool.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/TeamObserver.h"
#include "engine/SimplePlanTree.h"
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
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/RuleBook.h"

#include <iostream>

using std::shared_ptr;

namespace alica {


AlicaTime RunningPlan::assignmentProtectionTime = 0;

void RunningPlan::init() {
    assignmentProtectionTime = (((*supplementary::SystemConfig::getInstance())["Alica"]->get<unsigned long>(
                                      "Alica.AssignmentProtectionTime", NULL)) *
                                      1000000);
}

RunningPlan::RunningPlan(AlicaEngine* ae)
    : _ae(ae)
    , _planType(nullptr)
    , _plan(nullptr)
    , _activeState(nullptr)
    , _activeEntryPoint(nullptr)
    , _behaviour(false)
    , _planStartTime(0)
    , _stateStartTime(0)
    , _assignment(nullptr)
    , _status(PlanStatus::Running)
    , _failCount(0)
    , _basicBehaviour(nullptr)
    , _active(false)
    , _allocationNeeded(false)
    , _failHandlingNeeded(false)
    , _cycleManagement(std::make_shared<CycleManager>(ae, this))
{}

RunningPlan::~RunningPlan() {}

RunningPlan::RunningPlan(AlicaEngine* ae, const Plan* plan)
        : RunningPlan(ae) {
    _plan = plan;
    /*
    Code does not have any effect
    vector<EntryPoint*> epCol;
    transform(plan->getEntryPoints().begin(), plan->getEntryPoints().end(), back_inserter(epCol),
            [](map<long, EntryPoint*>::value_type& val) { return val.second; });

    sort(epCol.begin(), epCol.end(), EntryPoint::compareTo);
    */
}

RunningPlan::RunningPlan(AlicaEngine* ae, const PlanType* pt)
        : RunningPlan(ae) {
    _planType = pt;
}

RunningPlan::RunningPlan(AlicaEngine* ae, const BehaviourConfiguration* bc)
    : _ae(ae)
    , _planType(nullptr)
    , _plan(bc)
    , _activeState(nullptr)
    , _activeEntryPoint(nullptr)
    , _behaviour(true)
    , _planStartTime(0)
    , _stateStartTime(0)
    , _assignment(nullptr)
    , _status(PlanStatus::Running)
    , _failCount(0)
    , _basicBehaviour(nullptr)
    , _active(false)
    , _allocationNeeded(false)
    , _failHandlingNeeded(false)
    , _cycleManagement(std::make_shared<CycleManager>(ae, this))
        {}

/**
 * Indicates whether this plan needs failure handling
 */
bool RunningPlan::getFailHandlingNeeded() const {
    return _failHandlingNeeded;
}
void RunningPlan::setFailHandlingNeeded(bool failHandlingNeeded) {
    if (failHandlingNeeded) {
        _status = PlanStatus::Failed;
    } else {
        if (_status == PlanStatus::Failed) {
            _status = PlanStatus::Running;
        }
    }
    _failHandlingNeeded = failHandlingNeeded;
}

/**
 * Gets/Sets the parent RunningPlan of this RunningPlan. Null in case this is the top-level element.
 */
void RunningPlan::setParent(weak_ptr<RunningPlan> s) {
    _parent = s;
}
weak_ptr<RunningPlan> RunningPlan::getParent() const {
    return _parent;
}

/**
 * Called once per Engine iteration, performs all neccessary checks and executes rules from the rulebook.
 * @param rules a RuleBook
 * @return PlanChange a PlanChange
 */
PlanChange RunningPlan::tick(RuleBook* rules) {
    _cycleManagement->update();
    PlanChange myChange = rules->visit(shared_from_this());
    PlanChange childChange = PlanChange::NoChange;
    // attention: do not use for each here: children are modified
    for (int i = 0; i < _children.size(); i++) {
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
void RunningPlan::setAllocationNeeded(bool need) {
    _allocationNeeded = need;
}

/**
 * Evaluates the precondition of the associated plan.
 * @return Whether the precondition currently holds or not.
 */
bool RunningPlan::evalPreCondition() {
    if (_plan == nullptr) {
        cerr << "Cannot Eval Condition, Plan is null" << endl;
        throw new exception();
    }
    if (_plan->getPreCondition() == nullptr) {
        return true;
    }
    try {
        return _plan->getPreCondition()->evaluate(shared_from_this());
    } catch (exception& e) {
        cerr << "Exception in precondition: " << e.what() << endl;
        return false;
    }
}

/**
 * Evals the runtime condition of the associated plan.
 * @return Whether the runtime currently holds or not.
 */
bool RunningPlan::evalRuntimeCondition() {
    if (_plan == nullptr) {
        cerr << "Cannot Eval Condition, Plan is null" << endl;
        throw new exception();
    }
    if (_plan->getRuntimeCondition() == nullptr) {
        return true;
    }
    try {
        return _plan->getRuntimeCondition()->evaluate(shared_from_this());
    } catch (exception& e) {
        cerr << "Exception in runtimecondition: " << _plan->getName() << e.what() << endl;
        return false;
    }
}

void RunningPlan::setActiveState(const State* s) {
    if (_activeState != s) {
        _activeState = s;
        _stateStartTime = _ae->getIAlicaClock()->now();
        if (_activeState != nullptr) {
            if (_activeState->isFailureState()) {
                _status = PlanStatus::Failed;
            } else if (_activeState->isSuccessState()) {
                const supplementary::AgentID*  mid = getOwnID();
                _assignment->getEpSuccessMapping()->getRobots(_activeEntryPoint)->push_back(mid);
                _ae->getTeamManager()->setSuccess(mid, _plan, _activeEntryPoint);
            }
        }
    }
}

void RunningPlan::addChildren(shared_ptr<list<shared_ptr<RunningPlan>>>& runningPlans) {
    for (shared_ptr<RunningPlan> r : (*runningPlans)) {
        r->setParent(shared_from_this());
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

void RunningPlan::addChildren(list<shared_ptr<RunningPlan>>& children) {
    for (shared_ptr<RunningPlan> r : children) {
        r->setParent(shared_from_this());
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
void RunningPlan::moveState(const State* nextState) {
    deactivateChildren();
    clearChildren();
    _assignment->moveRobots(_activeState, nextState);
    setActiveState(nextState);
    _failedSubPlans.clear();
}



/**
 * The children of this RunningPlan.
 */
list<shared_ptr<RunningPlan>>* RunningPlan::getChildren() {
    return &_children;  // TODO irgendwie ist das doch eher uncool
}

void RunningPlan::setChildren(list<shared_ptr<RunningPlan>> children) {
    _children = children;
}


void RunningPlan::setPlan(const AbstractPlan* plan) {
    if (_plan != plan) {
        _planStartTime = _ae->getIAlicaClock()->now();
        revokeAllConstraints();
    }
    _plan = plan;
}

/**
 * The behaviour represented by this running plan, in case there is any, otherwise null.
 */
shared_ptr<BasicBehaviour> RunningPlan::getBasicBehaviour() {
    return _basicBehaviour;
}
void RunningPlan::setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour) {
    _basicBehaviour = basicBehaviour;
}

/**
 * Simple method to recursively print the plan-tree.
 */
void RunningPlan::printRecursive() {
    for (shared_ptr<RunningPlan>& c : _children) {
        c->printRecursive();
    }
    if (_children.size() > 0) {
        cout << "END CHILDREN of " << (_plan == nullptr ? "NULL" : _plan->getName()) << endl;
    }
}

/**
 * The current assignment of robots to EntryPoints.
 */
shared_ptr<Assignment> RunningPlan::getAssignment() const {
    return _assignment;
}

void RunningPlan::setAssignment(shared_ptr<Assignment> assignment) {
    _assignment = assignment;
}


void RunningPlan::setActive(bool active) {
    _active = active;
}

/**
 * The robot's current EntryPoint. Null if it is idling
 */
EntryPoint* RunningPlan::getOwnEntryPoint() const {
    return _activeEntryPoint;
}

/**
 * The robot's current EntryPoint. Null if it is idling
 */
void RunningPlan::setOwnEntryPoint(const EntryPoint* value) {
    if (_activeEntryPoint != value) {
        const supplementary::AgentID* mid = getOwnID();
        _assignment->removeRobot(mid);
        _activeEntryPoint = value;
        if (_activeEntryPoint != nullptr) {
            setActiveState(_activeEntryPoint->getState());
            _assignment->addRobot(mid, _activeEntryPoint, _activeState);
        }
    }
}

/**
 * Gets the PlanStatus of the currently executed plan.
 */
PlanStatus RunningPlan::getStatus() const {
    if (_basicBehaviour != nullptr) {
        if (_basicBehaviour->isSuccess()) {
            // cout << "RP: " << _plan->getName() << " BEH Success" << endl;
            return PlanStatus::Success;
        } else if (_basicBehaviour->isFailure()) {
            // cout << "RP: " << _plan->getName() << " BEH Failed" << endl;
            return PlanStatus::Failed;
        } else {
            // cout << "RP: " << _plan->getName() << " BEH Running" << endl;
            return PlanStatus::Running;
        }
    }
    if (_assignment != nullptr && _assignment->isSuccessfull()) {
        // cout << "RP: " << _plan->getName() << " ASS Success" << endl;
        return PlanStatus::Success;
    }
    // cout << "RP: " << _plan->getName() << " STATUS " << (_status == PlanStatus::Running ? "RUNNING" :
    // (_status == PlanStatus::Success ? "SUCCESS" : "FAILED")) << endl;
    return _status;
}

void RunningPlan::clearFailures() {
    _failCount = 0;
}

/**
 * Clears the failure history of failed plans.
 */
void RunningPlan::clearFailedChildren() {
    _failedSubPlans.clear();
}

void RunningPlan::addFailure() {
    _failCount++;
    _failHandlingNeeded = true;
}

/**
 * Returns the number of failures detected while this RunningPlan was executed.
 */
int RunningPlan::getFailure() {
    return _failCount;
}

void RunningPlan::deactivateChildren() {
    for (shared_ptr<RunningPlan>& r : _children) {
        r->deactivate();
    }
}

/**
 * Remove all children without passing any command to them.
 */
void RunningPlan::clearChildren() {
    _children.clear();
}

/**
 * Adapt the assignment of this plan to the one supplied. This can also change plan
 * @param r A RunningPlan
 */
void RunningPlan::adaptAssignment(shared_ptr<RunningPlan> r) {
    const State* newState = r->getAssignment()->getRobotStateMapping()->getState(getOwnID());
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
        auto robotsJoined = r->getAssignment()->getRobotStateMapping()->getRobotsInState(newState);
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


void RunningPlan::setActiveEntryPoint(EntryPoint* activeEntryPoint) {
    if (_activeEntryPoint != activeEntryPoint) {
        const supplementary::AgentID* mid = getOwnID();
        _assignment->removeRobot(mid);
        _activeEntryPoint = activeEntryPoint;
        if (_activeEntryPoint != nullptr) {
            setActiveState(_activeEntryPoint->getState());
            _assignment->addRobot(mid, _activeEntryPoint, _activeState);
        }
    }
}

/**
 * Indicates whether this running plan represents a behaviour.
 */
std::shared_ptr<CycleManager> RunningPlan::getCycleManagement() {
    return _cycleManagement;
}

/**
 * Indicate that an AbstractPlan has failed while being a child of this plan.
 * @param child a AbstractPlan
 */
void RunningPlan::setFailedChild(const AbstractPlan* child) {
    if (_failedSubPlans.find(child) != _failedSubPlans.end()) {
        _failedSubPlans.at(child)++;
    } else {
        _failedSubPlans.insert(pair<const AbstractPlan*, int>(child, 1));
    }
}

void RunningPlan::setRobotAvail(const supplementary::AgentID* robot) {
    auto iter = std::find_if(_robotsAvail.begin(), _robotsAvail.end(),
            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
    if (iter != _robotsAvail.end()) {
        return;
    }
    _robotsAvail.push_back(robot);
}

void RunningPlan::setRobotUnAvail(const supplementary::AgentID* robot) {
    std::remove_if(_robotsAvail.begin(), _robotsAvail.end(),
        [&robot](const supplementary::AgentID* id) { return *robot == *id; });
}

/**
 *  General Visitor pattern for the plan graph.
 *  @param vis A IPlanTreeVisitor
 */
void RunningPlan::accept(IPlanTreeVisitor* vis) {
    vis->visit(shared_from_this());
    //		for (int i = 0; i < _children.size(); i++)
    //		{
    //			auto iter = _children.begin();
    //			advance(iter, i);
    //			(*iter)->accept(vis);
    //		}
    for (std::shared_ptr<RunningPlan>& child : _children) {
        child->accept(vis);
    }
}

/**
 *  Deactivate this plan, to be called before the plan is removed from the graph.
 * Ensures that all sub-behaviours are stopped and all constraints are revoked.
 */
void RunningPlan::deactivate() {
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
bool RunningPlan::allChildrenStatus(PlanStatus ps) {
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
bool RunningPlan::anyChildrenTaskSuccess() {
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
                    childAssignment->getEpSuccessMapping()->getRobots()[i]->size() >=
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
void RunningPlan::activate() {
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
void RunningPlan::limitToRobots(const AgentSet& robots) {
    if (isBehaviour()) {
        return;
    }
    if (!_cycleManagement->mayDoUtilityCheck()) {
        return;
    }
    bool recurse = false;
    
    //TODO: move this code into assignment to avoid the call to getAllRobots.
    AgentSet curRobots;
    _assignment->getAllRobots(curRobots);
    for (auto& r : (*curRobots)) {
        if (find_if(robots->begin(), robots->end(),
                    [&r](const supplementary::AgentID* id) { return *r == *id; }) == curRobots->end()) {
            if (_activeState != nullptr &&
                    _assignment->getRobotStateMapping()->stateOfRobot(r) == _activeState) {
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
void RunningPlan::revokeAllConstraints() {
    _constraintStore.clear();
}

void RunningPlan::attachPlanConstraints() {
    _constraintStore.addCondition(_plan->getPreCondition());
    _constraintStore.addCondition(_plan->getRuntimeCondition());
}

bool RunningPlan::recursiveUpdateAssignment(list<shared_ptr<SimplePlanTree>> spts,
        std::vector<const supplementary::AgentID*>& availableAgents, list<const supplementary::AgentID*> noUpdates,
        AlicaTime now) {
    if (isBehaviour()) {
        return false;
    }
    bool keepTask = ((_planStartTime + assignmentProtectionTime) > now);
    bool auth = _cycleManagement->haveAuthority();

    // if keepTask, the task Assignment should not be changed!
    bool ret = false;
    AllocationDifference* aldif = new AllocationDifference();
    for (shared_ptr<SimplePlanTree> spt : spts) {
        if (spt->getState()->getInPlan() != _plan) {  // the robot is no longer participating in this plan
            if (!keepTask & !auth) {
                EntryPoint* ep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
                if (ep != nullptr) {
                    getAssignment()->removeRobot(spt->getRobotId());
                    ret = true;
                    aldif->editSubtractions().emplace_back(ep, spt->getRobotId());
                }
            }
        } else {
            if (keepTask || auth) {  // Update only state, and that only if it is in the reachability graph of its
                                     // current entrypoint, else
                // ignore
                EntryPoint* cep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
                if (cep != nullptr) {
                    if (cep->getReachableStates().find(spt->getState()) != cep->getReachableStates().end()) {
                        getAssignment()->getRobotStateMapping()->setState(spt->getRobotId(), spt->getState());
                    }
                } else {  // robot was not expected to be here during protected assignment time, add it.
                    getAssignment()->addRobot(spt->getRobotId(), spt->getEntryPoint(), spt->getState());
                    aldif->editAdditions().emplace_back(spt->getEntryPoint(), spt->getRobotId());
                }
            } else {  // Normal Update
                EntryPoint* ep = getAssignment()->getEntryPointOfRobot(spt->getRobotId());
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
    if (!keepTask) {  // remove any robot no longer available in the spts (auth flag obey here, as robot might be
                      // unavailable)
        // EntryPoint[] eps = this.Assignment.GetEntryPoints();
        EntryPoint* ep;
        const supplementary::AgentID* ownId = getOwnID();
        for (int i = 0; i < getAssignment()->getEntryPointCount(); i++) {
            ep = getAssignment()->getEpRobotsMapping()->getEp(i);
            rem.clear();
            auto robs = getAssignment()->getRobotsWorking(ep);
            for (auto& rob : (*robs)) {
                if (*rob == *ownId)
                    continue;
                bool found = false;
                if (find_if(noUpdates.begin(), noUpdates.end(),
                            [&rob](const supplementary::AgentID* id) { return *rob == *id; }) != noUpdates.end()) {
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
    if (!auth) {  // under authority do not remove robots from assignment
        EntryPoint* ep;
        for (int i = 0; i < getAssignment()->getEntryPointCount(); i++) {
            ep = getAssignment()->getEpRobotsMapping()->getEp(i);
            rem.clear();
            auto robs = getAssignment()->getRobotsWorking(ep);
            for (auto& rob : (*robs)) {
                if (find_if(availableAgents.begin(), availableAgents.end(), [&rob](const supplementary::AgentID* id) {
                        return *rob == *id;
                    }) == availableAgents.end()) {
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
        unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>
                robotsJoined = getAssignment()->getRobotStateMapping()->getRobotsInState(getActiveState());
        for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
            if (std::find_if(robotsJoined.begin(), robotsJoined.end(),
                        [&iter](const supplementary::AgentID* id) { return *(*iter) == *id; }) == robotsJoined.end()) {
                iter = availableAgents.erase(iter);
            } else {
                ++iter;
            }
        }
    } else if (auth) {  // in case of authority, remove all that are not assigned to same task
        std::shared_ptr<std::vector<const supplementary::AgentID*>> robotsJoined =
                getAssignment()->getRobotsWorking(getOwnEntryPoint());

        if (robotsJoined) {
            for (auto iter = availableAgents.begin(); iter != availableAgents.end();) {
                if (find_if(robotsJoined->begin(), robotsJoined->end(), [&iter](const supplementary::AgentID* id) {
                        return *(*iter) == *id;
                    }) == robotsJoined->end()) {
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

void RunningPlan::toMessage(list<long>& message, shared_ptr<const RunningPlan>& deepestNode, int& depth, int curDepth) const {
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

std::string RunningPlan::toString() const {
    std::stringstream ss;
    ss << "######## RP ##########" << endl;
    ss << "Plan: " + (_plan != nullptr ? _plan->getName() : "NULL") << endl;
    ss << "PlanType: " << (_planType != nullptr ? _planType->getName() : "NULL") << endl;
    ss << "ActState: " << (_activeState != nullptr ? _activeState->getName() : "NULL") << endl;
    ss << "Task: " << (getOwnEntryPoint() != nullptr ? getOwnEntryPoint()->getTask()->getName() : "NULL")
       << std::endl;
    ss << "IsBehaviour: " << isBehaviour() << "\t";
    if (isBehaviour()) {
        ss << "Behaviour: " << (_basicBehaviour == nullptr ? "NULL" : _basicBehaviour->getName()) << std::endl;
    }
    ss << "AllocNeeded: " << _allocationNeeded << std::endl;
    ss << "FailHandlingNeeded: " << _failHandlingNeeded << "\t";
    ss << "FailCount: " << _failCount << std::endl;
    ss << "IsActive: " << _active << std::endl;
    ss << "Status: "
       << (_status == PlanStatus::Running ? "RUNNING"
                                               : (_status == PlanStatus::Success ? "SUCCESS" : "FAILED"))
       << std::endl;
    ss << "AvailRobots: ";
    for (const supplementary::AgentID* r : _robotsAvail) {
        ss << " " << *r;
    }
    ss << "\n";
    if (_assignment != nullptr) {
        ss << "Assignment:" << _assignment->toString();
    } else
        ss << "Assignment is null." << std::endl;
    ss << "Children: " << _children.size();
    if (_children.size() > 0) {
        ss << " ( ";
        for (const shared_ptr<RunningPlan>& r : _children) {
            if (r->_plan == nullptr) {
                ss << "NULL PLAN, ";
            } else
                ss << r->_plan->getName() + ", ";
        }
        ss << ")";
    }
    ss << endl
       << "CycleManagement - Assignment Overridden: " << (_cycleManagement->isOverridden() ? "true" : "false")
       << endl;
    ss << "\n########## ENDRP ###########" << endl;
    return ss.str();
}

/**
 * Tests whether any child has a specific status.
 * @param A PlanStatus
 * @return bool
 */
bool RunningPlan::anyChildrenStatus(PlanStatus ps) {
    for (std::shared_ptr<RunningPlan>& child : _children) {
        if (ps == child->getStatus()) {
            return true;
        }
    }
    return false;
}


void RunningPlan::sendLogMessage(int level, string& message) {
    _ae->getCommunicator()->sendLogMessage(level, message);
}

} /* namespace alica */
