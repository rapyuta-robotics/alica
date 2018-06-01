#pragma once

//#define EXPANSIONEVAL
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

class IAssignment;
class Assignment;
class Plan;
class EntryPoint;
class PartialAssignment;
class SimplePlanTree;
class TeamManager;
class TeamObserver;
class PartialAssignmentPool;

/**
 * Represents an instance of an assignment problem for one plan or a plantype.
 * All parameters, which are static for this problem, are stored here.
 */
class TaskAssignment final : public ITaskAssignment
{
public:
    TaskAssignment(const AlicaEngine* engine, const PlanGrp& planList, const AgentGrp& paraRobots, bool preassignOtherRobots);
    virtual ~TaskAssignment();
    virtual Assignment getNextBestAssignment(const Assignment* oldAss) override;
    std::string toString() const;
#ifdef EXPANSIONEVAL
    int getExpansionCount();
    void setExpansionCount(int expansionCount);
#endif
private:
    PartialAssignment* calcNextBestPartialAssignment(const Assignment* oldAss);

    // Plan to build an assignment for
    TeamManager* tm;
    TeamObserver* to;
    PlanGrp planList;
    AgentGrp robots;
    std::vector<EntryPoint*> entryPointVector;
    // Fringe of the search tree
    std::vector<PartialAssignment*> fringe;
    bool addAlreadyAssignedRobots(PartialAssignment* pa,
            std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>* simplePlanTreeMap);

#ifdef EXPANSIONEVAL
    int expansionCount;
#endif
};

} /* namespace alica */
