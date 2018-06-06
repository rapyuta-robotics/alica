#pragma once
#include <engine/Types.h>
#include <engine/UtilityFunction.h>
#include <engine/UtilityInterval.h>
#include <engine/model/Plan.h>
#include <engine/planselector/DynCardinality.h>
#include <supplementary/AgentID.h>

#include <vector>

namespace alica
{
class PartialAssignmentPool;
class TaskAssignment;
class Assignment;
class SimplePlanTree;

class PartialAssignment final
{
public:
    PartialAssignment();
    ~PartialAssignment();
    void clear();
    void prepare(const Plan* p, const TaskAssignment* problem);
    bool isValid() const;
    bool isGoal() const;
    const Plan* getPlan() const { return _plan; }
    bool addIfAlreadyAssigned(const SimplePlanTree* spt, AgentIDConstPtr agent, int idx);
    bool assignUnassignedAgent(int agentIdx, int epIdx);
    UtilityInterval getUtility() const { return _utility; }
    const TaskAssignment* getProblem() const { return _problem; }
    int getAssignedAgentCount() const { return _numAssignedAgents; }
    int getTotalAgentCount() const { return _assignment.size(); }
    int getEntryPointIndexOf(int agentIdx) const { return _assignment[agentIdx]; }
    const SuccessCollection* getSuccessData() const;

    bool expand(std::vector<PartialAssignment*>& o_container, PartialAssignmentPool& pool, const Assignment* old);
    void evaluate(const Assignment* old) { _utility = _plan->getUtilityFunction()->eval(this, old); }
    static bool compare(const PartialAssignment* a, const PartialAssignment* b);

    /* short getEntryPointCount() const override;
     const AgentGrp* getRobotsWorking(const EntryPoint* ep) const override;
     const AgentGrp* getRobotsWorking(int64_t epid) const override;
     std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) override;



     std::string toString() const override;
     virtual AssignmentCollection* getEpRobotsMapping() const override { return epRobotsMapping; }

     std::shared_ptr<UtilityFunction> getUtilFunc() const { return utilFunc; }
     virtual std::shared_ptr<SuccessCollection> getEpSuccessMapping() const override { return epSuccessMapping; }
     std::string assignmentCollectionToString() const override;
     void setMax(double max);
     const AgentGrp& getRobotIds() const;*/

private:
    friend std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);
    const Plan* _plan;
    const TaskAssignment* _problem;
    std::vector<DynCardinality> _cardinalities;
    std::vector<int> _assignment;
    UtilityInterval _utility;
    int _numAssignedAgents;
    int _nextAgentIdx;

    static bool s_allowIdling;
};

std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);

} /* namespace alica */
