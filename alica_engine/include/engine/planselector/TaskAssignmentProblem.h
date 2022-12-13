#pragma once

#define EXPANSIONEVAL

#include "engine/ITaskAssignmentProblem.h"
#include "engine/Types.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/planselector/PartialAssignmentPool.h"

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace alica
{
class Assignment;
class PartialAssignment;
class SimplePlanTree;
class TeamManager;
class TeamObserver;
class Blackboard;

/**
 * Represents an instance of an assignment problem for one plan or a plantype.
 * All parameters, which are static for this problem, are stored here.
 */
class TaskAssignmentProblem final : public ITaskAssignmentProblem
{
public:
    TaskAssignmentProblem(const TeamObserver& teamObserver, const TeamManager& teamManager, const PlanGrp& planList, const AgentGrp& paraAgents,
            PartialAssignmentPool& pool, const Blackboard* worldModels);
    virtual ~TaskAssignmentProblem();
    void preassignOtherAgents();

    Assignment getNextBestAssignment(const Assignment* oldAss) override;

#ifdef EXPANSIONEVAL
    int getExpansionCount() const { return _expansionCount; }
    void setExpansionCount(int expansionCount) { _expansionCount = expansionCount; }
#endif

    int getAgentCount() const { return _agents.size(); }
    const AgentGrp& getAgents() const { return _agents; }

    const SuccessCollection* getSuccessData(const Plan* p) const
    {
        for (int i = 0; i < static_cast<int>(_plans.size()); ++i) {
            if (_plans[i] == p) {
                return &_successData[i];
            }
        }
        return nullptr;
    }

private:
    friend std::ostream& operator<<(std::ostream& out, const TaskAssignmentProblem& tap);

    PartialAssignment* calcNextBestPartialAssignment(const Assignment* oldAss);

    const TeamManager& _teamManager;
    const TeamObserver& _teamObserver;
    PartialAssignmentPool& _pool;
    PlanGrp _plans;
    AgentGrp _agents;
    std::vector<SuccessCollection> _successData;
    const Blackboard* _worldModels;

    // Fringe of the search tree
    std::vector<PartialAssignment*> _fringe;
    bool addAlreadyAssignedRobots(PartialAssignment* pa, const std::map<AgentId, std::unique_ptr<SimplePlanTree>>& simplePlanTreeMap);

#ifdef EXPANSIONEVAL
    int _expansionCount;
#endif
};
std::ostream& operator<<(std::ostream& out, const TaskAssignmentProblem& tap);

} /* namespace alica */
