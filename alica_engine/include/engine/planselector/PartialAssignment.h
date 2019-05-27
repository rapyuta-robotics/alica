#pragma once
#include <engine/Types.h>
#include <engine/UtilityFunction.h>
#include <engine/UtilityInterval.h>
#include <engine/model/Plan.h>

#include <alica_solver_interface/Interval.h>

#include <vector>

namespace alica
{
class PartialAssignmentPool;
class TaskAssignmentProblem;
class Assignment;
class SimplePlanTree;

using DynCardinality = Interval<int>;

class PartialAssignment final
{
public:
    PartialAssignment();
    ~PartialAssignment();
    void clear();
    void prepare(const Plan* p, const TaskAssignmentProblem* problem);
    bool isValid() const;
    bool isGoal() const;
    const Plan* getPlan() const { return _plan; }
    bool addIfAlreadyAssigned(const SimplePlanTree* spt, essentials::IdentifierConstPtr agent, int idx);
    bool assignUnassignedAgent(int agentIdx, int epIdx);
    UtilityInterval getUtility() const { return _utility; }
    const TaskAssignmentProblem* getProblem() const { return _problem; }
    int getAssignedAgentCount() const { return _numAssignedAgents; }
    int getAssignedAgentCount(int idx) const;
    int getTotalAgentCount() const { return _assignment.size(); }
    int getEntryPointIndexOf(int agentIdx) const { return _assignment[agentIdx]; }
    int getEntryPointCount() const { return _cardinalities.size(); }
    const SuccessCollection* getSuccessData() const;

    bool expand(std::vector<PartialAssignment*>& o_container, PartialAssignmentPool& pool, const Assignment* old);
    void evaluate(const Assignment* old) { _utility = _plan->getUtilityFunction()->eval(this, old); }
    static bool compare(const PartialAssignment* a, const PartialAssignment* b);

    static void allowIdling(bool allowed) { s_allowIdling = allowed; }
    static bool isIdlingAllowed() { return s_allowIdling; }

private:
    friend std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);
    const Plan* _plan;
    const TaskAssignmentProblem* _problem;
    std::vector<DynCardinality> _cardinalities;
    std::vector<int> _assignment;
    UtilityInterval _utility;
    int _numAssignedAgents;
    int _nextAgentIdx;

    static bool s_allowIdling;
};

std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);

} /* namespace alica */
