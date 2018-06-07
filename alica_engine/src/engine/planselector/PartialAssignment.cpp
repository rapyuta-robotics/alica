#include "engine/planselector/PartialAssignment.h"

#include <engine/SimplePlanTree.h>
#include <engine/collections/SuccessCollection.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/Task.h>
#include <engine/planselector/DynCardinality.h>
#include <engine/planselector/PartialAssignmentPool.h>
#include <engine/planselector/TaskAssignmentProblem.h>
namespace alica
{
namespace
{
// constexpr int INFINITE = std::numeric_limits<int>::max();
constexpr int64_t PRECISION = 0x40000000;
}

bool PartialAssignment::s_allowIdling = true;

PartialAssignment::PartialAssignment()
        : _plan(nullptr)
        , _problem(nullptr)
        , _numAssignedAgents(0)
        , _nextAgentIdx(0)
        , _utility(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max())
{
}

PartialAssignment::~PartialAssignment() {}

/**
 * Checks whether the current assignment is valid
 */
bool PartialAssignment::isValid() const
{
    int min = 0;
    for (const DynCardinality& dc : _cardinalities) {
        min += dc.getMin();
    }
    return min <= _problem->getAgentCount() - _numAssignedAgents;
}

/**
 * Checks if this PartialAssignment is a complete Assignment.
 * @return True, if it is, false otherwise.
 */
bool PartialAssignment::isGoal() const
{
    // There should be no unassigned agents anymore
    if (_problem->getAgentCount() != _numAssignedAgents) {
        return false;
    }
    // Every EntryPoint should be satisfied according to his minCar
    for (const DynCardinality& dc : _cardinalities) {
        if (dc.getMin() >= 0) {
            return false;
        }
    }
    return true;
}
const SuccessCollection* PartialAssignment::getSuccessData() const
{
    return _problem->getSuccessData(_plan);
}

void PartialAssignment::clear()
{
    _plan = nullptr;
    _problem = nullptr;
    _numAssignedAgents = 0;
    _nextAgentIdx = 0;
    _cardinalities.clear();
    _assignment.clear();
    _utility = UtilityInterval(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
}

void PartialAssignment::prepare(const Plan* p, const TaskAssignmentProblem* problem)
{
    _plan = p;
    _problem = problem;
    _numAssignedAgents = 0;
    _assignment.clear();
    _assignment.resize(problem->getAgentCount(), -1);
    _cardinalities.reserve(p->getEntryPoints().size());
    for (const EntryPoint* ep : p->getEntryPoints()) {
        _cardinalities.push_back(ep->getCardinality());
    }
}

bool PartialAssignment::assignUnassignedAgent(int agentIdx, int epIdx)
{
    if (_cardinalities[epIdx].getMax() > 0) {
        --_cardinalities[epIdx];
        assert(_assignment[agentIdx] < 0); // we assume the agent was unassigned
        _assignment[agentIdx] = epIdx;
        if (_nextAgentIdx == agentIdx) {
            ++_nextAgentIdx;
        }
        return true;
    }
    return false;
}
/**
 * If the robot has already assigned itself, this method updates the partial assignment accordingly
 */
bool PartialAssignment::addIfAlreadyAssigned(const SimplePlanTree* spt, AgentIDConstPtr agent, int idx)
{
    if (spt->getEntryPoint()->getPlan() == _plan) {

        for (int i = 0; i < static_cast<int>(_cardinalities.size()); ++i) {
            const EntryPoint* curEp = _plan->getEntryPoints()[i];
            if (spt->getEntryPoint()->getId() == curEp->getId()) {
                return assignUnassignedAgent(idx, i);
            }
        }
        return false;
    }
    // If there are children and we didnt find the robot until now, then go on recursive
    else {
        for (const std::shared_ptr<SimplePlanTree>& sptChild : spt->getChildren()) {
            if (addIfAlreadyAssigned(sptChild.get(), agent, idx)) {
                return true;
            }
        }
    }
    // Did not find the robot in any relevant entry point
    return false;
}

std::ostream& operator<<(std::ostream& out, const PartialAssignment& pa)
{
    const Plan* p = pa._plan;
    out << "Plan: " << (p != nullptr ? p->getName() : "NULL") << std::endl;
    out << "Utility: " << pa._utility << std::endl;
    out << "Agents: ";
    for (AgentIDConstPtr agent : pa._problem->getAgents()) {
        out << *agent << " ";
    }
    out << std::endl;
    if (p) {
        for (int i = 0; i < static_cast<int>(pa._cardinalities.size()); ++i) {
            out << "EPid: " << p->getEntryPoints()[i]->getId() << " Task: " << p->getEntryPoints()[i]->getTask()->getName()
                << " cardinality: " << pa._cardinalities[i];
        }
    }
    out << std::endl;
    out << " Assigned Robots: " << std::endl;
    int i = 0;
    for (int idx : pa._assignment) {
        out << "Agent: " << pa._problem->getAgents()[i] << " Ep: " << idx;
    }
    out << std::endl;
    //    out << "HashCode: " << this->getHash() << std::endl;
    return out;
}
bool PartialAssignment::expand(std::vector<PartialAssignment*>& o_container, PartialAssignmentPool& pool, const Assignment* old)
{
    // iterate next idx for cases of pre-assigned agents:
    while (_nextAgentIdx < static_cast<int>(_cardinalities.size()) && _assignment[_nextAgentIdx] >= 0) {
        ++_nextAgentIdx;
    }
    if (_nextAgentIdx >= static_cast<int>(_cardinalities.size())) {
        // No robot left to expand
        return false;
    }
    bool change = false;
    const int numChildren = static_cast<int>(_cardinalities.size());
    for (int i = 0; i < numChildren; ++i) {
        if (_cardinalities[i].getMax() > 0) {
            PartialAssignment* newPa = pool.getNext();
            *newPa = *this;
            newPa->assignUnassignedAgent(_nextAgentIdx, i);
            newPa->evaluate(old);
            if (newPa->_utility.getMax() > -1.0) {
                o_container.insert(std::upper_bound(o_container.begin(), o_container.end(), newPa, compare), newPa);
                change = true;
            }
        }
    }
    if (s_allowIdling) {
        PartialAssignment* newPa = pool.getNext();
        *newPa = *this;
        newPa->assignUnassignedAgent(_nextAgentIdx, _cardinalities.size());
        newPa->evaluate(old);
        if (newPa->_utility.getMax() > -1.0) {
            o_container.insert(std::upper_bound(o_container.begin(), o_container.end(), newPa, compare), newPa);
            change = true;
        }
    }
    return change;
}

bool PartialAssignment::compare(const PartialAssignment* a, const PartialAssignment* b)
{
    // TODO has perhaps to be changed
    // 0 , -1 = false
    // 1 true
    if (a == b) {
        return false;
    }
    assert(a->getProblem() == b->getProblem());
    const int64_t aval = static_cast<int64_t>(std::round(a->getUtility().getMax() * PRECISION));
    const int64_t bval = static_cast<int64_t>(std::round(b->getUtility().getMax() * PRECISION));
    if (aval > bval) {
        // a has higher possible utility
        return true;
    } else if (aval < bval) {
        // b has higher possible utility
        return false;
    }
    // Now we are sure that both partial assignments have the same utility
    else if (a->getPlan()->getId() < b->getPlan()->getId()) {
        return false;
    } else if (b->getPlan()->getId() > b->getPlan()->getId()) {
        return true;
    }
    // Now we are sure that both partial assignments have the same utility and the same plan id
    if (a->getAssignedAgentCount() > b->getAssignedAgentCount()) {
        return true;
    } else if (a->getAssignedAgentCount() < b->getAssignedAgentCount()) {
        return false;
    }
    if (a->getUtility().getMin() > b->getUtility().getMin()) {
        // other has higher actual utility
        return true;
    } else if (a->getUtility().getMin() < b->getUtility().getMin()) {
        // this has higher actual utility
        return false;
    }
    for (int i = 0; i < static_cast<int>(a->_assignment.size()); ++i) {
        if (a->_assignment[i] > b->_assignment[i]) {
            return true;
        } else if (a->_assignment[i] < b->_assignment[i]) {
            return false;
        }
    }
    return false;
}

} /* namespace alica */
/*

void PartialAssignment::clear()
{
    this->min = 0.0;
    this->max = 0.0;
    this->compareVal = PRECISION;
    this->unassignedRobotIds.clear();
    this->epRobotsMapping->clear();
    _hash = 0;
}

void PartialAssignment::reset(PartialAssignmentPool* pap)
{
    pap->curIndex = 0;
}

const AgentGrp& PartialAssignment::getRobotIds() const
{
    return robotIds;
}

PartialAssignment* PartialAssignment::getNew(PartialAssignmentPool* pap, const AgentGrp& robotIds, const Plan* plan, shared_ptr<SuccessCollection> sucCol)
{
    if (pap->curIndex >= pap->maxCount) {
        std::cerr << "max PA count reached!" << std::endl;
    }
    PartialAssignment* ret = pap->daPAs[pap->curIndex++];
    ret->clear();
    ret->robotIds = robotIds; // Should already be sorted! (look at TaskAssignment, or PlanSelector)
    ret->plan = plan;
    ret->utilFunc = plan->getUtilityFunction();
    ret->epSuccessMapping = sucCol;
    // Create EP-Array
    if (AssignmentCollection::allowIdling) {
        ret->epRobotsMapping->setSize(plan->getEntryPoints().size() + 1);
        // Insert IDLE-EntryPoint
        ret->epRobotsMapping->setEp(ret->epRobotsMapping->getSize() - 1, pap->idleEP);
    } else {
        ret->epRobotsMapping->setSize(plan->getEntryPoints().size());
    }
    // Insert plan entrypoints
    int i = 0;
    for (const EntryPoint* ep : plan->getEntryPoints()) {
        ret->epRobotsMapping->setEp(i++, ep);
    }

    // Sort the entrypoint array
    ret->epRobotsMapping->sortEps();

    for (int i = 0; i < ret->epRobotsMapping->getSize(); i++) {
        ret->dynCardinalities[i]->setMin(ret->epRobotsMapping->getEp(i)->getMinCardinality());
        ret->dynCardinalities[i]->setMax(ret->epRobotsMapping->getEp(i)->getMaxCardinality());
        shared_ptr<list<const supplementary::AgentID*>> suc = sucCol->getRobots(ret->epRobotsMapping->getEp(i));

        if (suc != nullptr) {
            ret->dynCardinalities[i]->setMin(ret->dynCardinalities[i]->getMin() - suc->size());
            ret->dynCardinalities[i]->setMax(ret->dynCardinalities[i]->getMax() - suc->size());
            if (ret->dynCardinalities[i]->getMin() < 0) {
                ret->dynCardinalities[i]->setMin(0);
            }
            if (ret->dynCardinalities[i]->getMax() < 0) {
                ret->dynCardinalities[i]->setMax(0);
            }

#ifdef SUCDEBUG
            std::cout << "SuccessCollection" << std::endl;
            std::cout << "EntryPoint: " << ret->epRobotsMapping->getEntryPoints()->at(i)->toString() << std::endl;
            std::cout << "DynMax: " << ret->dynCardinalities[i]->getMax() << std::endl;
            std::cout << "DynMin: " << ret->dynCardinalities[i]->getMin() << std::endl;
            std::cout << "SucCol: ";
            for (int j : (*suc)) {
                std::cout << j << ", ";
            }
            std::cout << "-----------" << std::endl;
#endif
        }
    }

    // At the beginning all robots are unassigned
    for (const supplementary::AgentID* robotId : robotIds) {
        ret->unassignedRobotIds.push_back(robotId);
    }
    return ret;
}

PartialAssignment* PartialAssignment::getNew(PartialAssignmentPool* pap, PartialAssignment* oldPA)
{
    if (pap->curIndex >= pap->maxCount) {
        std::cerr << "max PA count reached!" << std::endl;
    }
    PartialAssignment* ret = pap->daPAs[pap->curIndex++];
    ret->clear();
    ret->min = oldPA->min;
    ret->max = oldPA->max;
    ret->plan = oldPA->plan;
    ret->robotIds = oldPA->robotIds;
    ret->utilFunc = oldPA->utilFunc;
    ret->epSuccessMapping = oldPA->epSuccessMapping;
    for (int i = 0; i < static_cast<int>(oldPA->unassignedRobotIds.size()); i++) {
        ret->unassignedRobotIds.push_back(oldPA->unassignedRobotIds[i]);
    }

    for (int i = 0; i < static_cast<int>(oldPA->dynCardinalities.size()); i++) {
        ret->dynCardinalities[i] = std::make_shared<DynCardinality>(oldPA->dynCardinalities[i]->getMin(), oldPA->dynCardinalities[i]->getMax());
    }
    *ret->epRobotsMapping = *oldPA->epRobotsMapping;

    return ret;
}

short PartialAssignment::getEntryPointCount() const
{
    return this->epRobotsMapping->getSize();
}

int PartialAssignment::totalRobotCount() const
{
    int c = 0;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        c += this->epRobotsMapping->getRobots(i)->size();
    }
    return this->getNumUnAssignedRobotIds() + c;
}

const std::vector<const supplementary::AgentID*>* PartialAssignment::getRobotsWorking(const EntryPoint* ep) const
{
    return this->epRobotsMapping->getRobotsByEp(ep);
}

const std::vector<const supplementary::AgentID*>* PartialAssignment::getRobotsWorking(int64_t epid) const
{
    return this->epRobotsMapping->getRobotsByEpId(epid);
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getRobotsWorkingAndFinished(const EntryPoint* ep)
{
    shared_ptr<list<const supplementary::AgentID*>> ret = std::make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robotIds = this->epRobotsMapping->getRobotsByEp(ep);
    if (robotIds != nullptr) {
        for (auto iter : (*robotIds)) {
            ret->push_back(iter);
        }
    }
    auto successes = this->epSuccessMapping->getRobots(ep);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            ret->push_back(iter);
        }
    }
    return ret;
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getRobotsWorkingAndFinished(int64_t epid)
{
    shared_ptr<list<const supplementary::AgentID*>> ret = std::make_shared<list<const supplementary::AgentID*>>();
    auto robots = this->epRobotsMapping->getRobotsByEpId(epid);
    if (robots != nullptr) {
        for (auto iter : (*robots)) {
            ret->push_back(iter);
        }
    }
    auto successes = this->epSuccessMapping->getRobotsById(epid);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            ret->push_back(iter);
        }
    }
    return ret;
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getUniqueRobotsWorkingAndFinished(const EntryPoint* ep)
{
    auto ret = std::make_shared<std::list<const supplementary::AgentID*>>();
    auto robots = this->epRobotsMapping->getRobotsByEp(ep);

    for (auto iter : (*robots)) {
        ret->push_back(iter);
    }

    auto successes = this->epSuccessMapping->getRobots(ep);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            if (std::find_if(ret->begin(), ret->end(), [&iter](const supplementary::AgentID* id) { return *iter == *id; }) == ret->end()) {
                ret->push_back(iter);
            }
        }
    }
    return ret;
}
*/

/**
 * Assigns the robot into the data structures according to the given index.
 * @return True, when it was possible to assign the robot. False, otherwise.
 */
/*bool PartialAssignment::assignRobot(const supplementary::AgentID* robotId, int index)
{
    if (this->dynCardinalities[index]->getMax() > 0) {
        this->epRobotsMapping->assignRobot(index, robotId);
        if (this->dynCardinalities[index]->getMin() > 0) {
            this->dynCardinalities[index]->setMin(this->dynCardinalities[index]->getMin() - 1);
        }
        if (this->dynCardinalities[index]->getMax() != INFINITE) {
            this->dynCardinalities[index]->setMax(this->dynCardinalities[index]->getMax() - 1);
        }
        return true;
    }
    return false;
}
*/
/*
shared_ptr<list<PartialAssignment*>> PartialAssignment::expand()
{
    shared_ptr<list<PartialAssignment*>> newPas = std::make_shared<list<PartialAssignment*>>();
    if (this->unassignedRobotIds.size() == 0) {
        // No robot left to expand
        return newPas;
    }
    // Robot which should be assigned next
    const supplementary::AgentID* robot = this->unassignedRobotIds[0];
    this->unassignedRobotIds.erase(this->unassignedRobotIds.begin());
    PartialAssignment* newPa;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        if (this->dynCardinalities[i]->getMax() > 0) {
            // Update the cardinalities and assign the robot
            newPa = PartialAssignment::getNew(pap, this);
            newPa->assignRobot(robot, i);
            newPas->push_back(newPa);
        }
    }
    return newPas;
}
*/

/**
 * Compares two partial assignments
 * @return false if it is the same object or they have the same utility, assignment and plan id
 * false if this PartialAssignment has a higher utility, or plan id
 * Difference between Hashcodes, if they have the same utility and plan id
 * true if the other PartialAssignment has a higher utility, or plan id
 */

/*
std::string PartialAssignment::toString() const
{
    std::stringstream ss;

    ss << "Plan: " << this->plan->getName() << std::endl;
    ss << "Utility: " << this->min << ".." << this->max << std::endl;
    ss << "unassignedRobots: ";
    for (auto& robot : this->unassignedRobotIds) {
        ss << robot << " ";
    }
    ss << std::endl;

    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        ss << "EPid: " << this->epRobotsMapping->getEp(i)->getId() << " Task: " << this->epRobotsMapping->getEp(i)->getTask()->getName()
           << " minCar: " << this->dynCardinalities[i]->getMin()
           << " maxCar: " << (this->dynCardinalities[i]->getMax() == INFINITE ? "*" : std::to_string(this->dynCardinalities[i]->getMax()))
           << " Assigned Robots: ";
        for (auto& robot : *this->epRobotsMapping->getRobots(i)) {
            ss << robot << " ";
        }
        ss << std::endl;
    }

    ss << this->epRobotsMapping->toString();
    ss << "HashCode: " << this->getHash() << std::endl;
    return ss.str();
}

std::string PartialAssignment::assignmentCollectionToString() const
{
    return "PA: \n" + toString();
}

void PartialAssignment::setMax(double max)
{
    this->max = max;
    this->compareVal = (long)round(max * PRECISION);
}
*/
