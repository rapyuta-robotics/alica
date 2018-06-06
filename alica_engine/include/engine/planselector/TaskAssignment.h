#pragma once

#define EXPANSIONEVAL
//#define TA_DEBUG

#include "engine/ITaskAssignment.h"
#include "engine/Types.h"
#include "supplementary/AgentID.h"
#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace alica
{
class AlicaEngine;
class Assignment;
class PartialAssignment;
class SimplePlanTree;
class TeamManager;
class TeamObserver;

/**
 * Represents an instance of an assignment problem for one plan or a plantype.
 * All parameters, which are static for this problem, are stored here.
 */
class TaskAssignment final : public ITaskAssignment
{
public:
    TaskAssignment(const AlicaEngine* engine, const PlanGrp& planList, const AgentGrp& paraAgents, PartialAssignmentPool& pool);
    virtual ~TaskAssignment();
    void preassignOtherAgents();

    virtual Assignment getNextBestAssignment(const Assignment* oldAss) override;

#ifdef EXPANSIONEVAL
    int getExpansionCount() const { return _expansionCount; }
    void setExpansionCount(int expansionCount) { _expansionCount = expansionCount; }
#endif

    int getAgentCount() const { return _agents.size(); }
    const AgentGrp& getAgents() const { return _agents; }

    std::string toString() const;
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
    PartialAssignment* calcNextBestPartialAssignment(const Assignment* oldAss);

    TeamManager* _tm;
    TeamObserver* _to;
    PartialAssignmentPool& _pool;
    PlanGrp _plans;
    AgentGrp _agents;
    std::vector<SuccessCollection> _successData;

    // Fringe of the search tree
    std::vector<PartialAssignment*> _fringe;
    bool addAlreadyAssignedRobots(PartialAssignment* pa,
            std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>* simplePlanTreeMap);

#ifdef EXPANSIONEVAL
    int _expansionCount;
#endif
};

} /* namespace alica */
