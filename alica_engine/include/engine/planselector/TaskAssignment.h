#pragma once

//#define EXPANSIONEVAL
//#define TA_DEBUG

#include "supplementary/AgentID.h"
#include "engine/ITaskAssignment.h"
#include "engine/Types.h"
#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <memory>
#include <sstream>
#include <string>
#include <vector>


namespace alica {

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
class TaskAssignment final : public ITaskAssignment {
public:
    TaskAssignment(const AlicaEngine* engine, const PlanSet& planList,
            const AgentSet& paraRobots, bool preassignOtherRobots);
    virtual ~TaskAssignment();
    std::shared_ptr<Assignment> getNextBestAssignment(IAssignment* oldAss);
    std::string toString();
#ifdef EXPANSIONEVAL
    int getExpansionCount();
    void setExpansionCount(int expansionCount);
#endif
private:
    PartialAssignment* calcNextBestPartialAssignment(IAssignment* oldAss);

    // Plan to build an assignment for
    TeamManager* tm;
    TeamObserver* to;
    PlanSet planList;
    AgentSet robots;
    std::vector<EntryPoint*> entryPointVector;
    // Fringe of the search tree
    std::vector<PartialAssignment*> fringe;
    bool addAlreadyAssignedRobots(PartialAssignment* pa,
            std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>*
                    simplePlanTreeMap);

#ifdef EXPANSIONEVAL
    int expansionCount;
#endif
};

} /* namespace alica */
