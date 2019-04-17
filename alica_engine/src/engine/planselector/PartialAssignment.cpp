#include "engine/planselector/PartialAssignment.h"

#include <engine/SimplePlanTree.h>
#include <engine/collections/SuccessCollection.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/Task.h>
#include <engine/planselector/PartialAssignmentPool.h>
#include <engine/planselector/TaskAssignmentProblem.h>

#include <cmath>

namespace alica
{
namespace
{
// constexpr int INFINITE = std::numeric_limits<int>::max();
constexpr int64_t PRECISION = 0x40000000;
} // namespace

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
        if (dc.getMax() < 0) {
            return false;
        }
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
        if (dc.getMin() > 0) {
            return false;
        }
    }
    return true;
}
const SuccessCollection* PartialAssignment::getSuccessData() const
{
    return _problem->getSuccessData(_plan);
}

int PartialAssignment::getAssignedAgentCount(int idx) const
{
    return _plan->getEntryPoints()[idx]->getCardinality().getMax() - _cardinalities[idx].getMax();
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
    _nextAgentIdx = 0;
    _assignment.clear();
    _assignment.resize(problem->getAgentCount(), -1);
    _cardinalities.clear();
    _cardinalities.reserve(p->getEntryPoints().size() + s_allowIdling ? 1 : 0);
    for (const EntryPoint* ep : p->getEntryPoints()) {
        _cardinalities.push_back(ep->getCardinality() - static_cast<int>(problem->getSuccessData(p)->getAgents(ep)->size()));
    }
    if (s_allowIdling) {
        _cardinalities.emplace_back(0, std::numeric_limits<int>::max());
    }
    _utility = UtilityInterval(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
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
        ++_numAssignedAgents;
        return true;
    }
    return false;
}
/**
 * If the robot has already assigned itself, this method updates the partial assignment accordingly
 */
// TODO: this is pretty inefficient
bool PartialAssignment::addIfAlreadyAssigned(const SimplePlanTree* spt, essentials::AgentIDConstPtr agent, int idx)
{
    if (spt->getEntryPoint()->getPlan() == _plan) {
        const int numEps = static_cast<int>(_plan->getEntryPoints().size());
        for (int i = 0; i < numEps; ++i) {
            const EntryPoint* curEp = _plan->getEntryPoints()[i];
            if (spt->getEntryPoint()->getId() == curEp->getId()) {
                return assignUnassignedAgent(idx, i);
            }
        }
        return false;
    }
    // If there are children and we didnt find the robot until now, then go on recursive
    else {
        for (const std::unique_ptr<SimplePlanTree>& sptChild : spt->getChildren()) {
            if (addIfAlreadyAssigned(sptChild.get(), agent, idx)) {
                return true;
            }
        }
    }
    // Did not find the robot in any relevant entry point
    return false;
}

bool PartialAssignment::expand(std::vector<PartialAssignment*>& o_container, PartialAssignmentPool& pool, const Assignment* old)
{
    // iterate next idx for cases of pre-assigned agents:
    while (_nextAgentIdx < static_cast<int>(_assignment.size()) && _assignment[_nextAgentIdx] >= 0) {
        ++_nextAgentIdx;
    }
    if (_nextAgentIdx >= static_cast<int>(_assignment.size())) {
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
    return change;
}

bool PartialAssignment::compare(const PartialAssignment* a, const PartialAssignment* b)
{
    if (a == b) {
        return false;
    }
    assert(a->getProblem() == b->getProblem());
    const int64_t aval = static_cast<int64_t>(std::round(a->getUtility().getMax() * PRECISION));
    const int64_t bval = static_cast<int64_t>(std::round(b->getUtility().getMax() * PRECISION));
    if (aval < bval) {
        // b has higher possible utility
        return true;
    } else if (aval > bval) {
        // a has higher possible utility
        return false;
    }
    // Now we are sure that both partial assignments have the same utility
    else if (a->getPlan()->getId() < b->getPlan()->getId()) {
        return false;
    } else if (b->getPlan()->getId() > b->getPlan()->getId()) {
        return true;
    }
    // Now we are sure that both partial assignments have the same utility and the same plan id
    if (a->getAssignedAgentCount() < b->getAssignedAgentCount()) {
        return true;
    } else if (a->getAssignedAgentCount() > b->getAssignedAgentCount()) {
        return false;
    }
    if (a->getUtility().getMin() < b->getUtility().getMin()) {
        // other has higher actual utility
        return true;
    } else if (a->getUtility().getMin() > b->getUtility().getMin()) {
        // this has higher actual utility
        return false;
    }
    for (int i = 0; i < static_cast<int>(a->_assignment.size()); ++i) {
        if (a->_assignment[i] < b->_assignment[i]) {
            return true;
        } else if (a->_assignment[i] > b->_assignment[i]) {
            return false;
        }
    }
    return false;
}

std::ostream& operator<<(std::ostream& out, const PartialAssignment& pa)
{
    const Plan* p = pa._plan;
    out << "Plan: " << (p != nullptr ? p->getName() : "NULL") << std::endl;
    out << "Utility: " << pa._utility << std::endl;
    out << "Agents: ";
    for (essentials::AgentIDConstPtr agent : pa._problem->getAgents()) {
        out << *agent << " ";
    }
    out << std::endl;
    if (p) {
        for (int i = 0; i < static_cast<int>(pa._cardinalities.size()) - PartialAssignment::s_allowIdling ? 1 : 0; ++i) {
            out << "EPid: " << p->getEntryPoints()[i]->getId() << " Task: " << p->getEntryPoints()[i]->getTask()->getName()
                << " cardinality: " << pa._cardinalities[i];
        }
    }
    out << std::endl;
    out << " Assigned Agents: " << std::endl;
    int i = 0;
    for (int idx : pa._assignment) {
        out << "Agent: " << pa._problem->getAgents()[i] << " Ep: " << idx << std::endl;
        ++i;
    }
    out << std::endl;
    return out;
}

} /* namespace alica */
