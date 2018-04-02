#pragma once

//#define EXPANSIONEVAL
//#define TA_DEBUG

#include "supplementary/AgentID.h"
#include "engine/ITaskAssignment.h"
#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
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
class TaskAssignment : virtual public ITaskAssignment {
public:
    TaskAssignment(const AlicaEngine* engine, list<Plan*> planList,
            shared_ptr<vector<const supplementary::AgentID*>> paraRobots, bool preassignOtherRobots);
    virtual ~TaskAssignment();
    shared_ptr<Assignment> getNextBestAssignment(IAssignment* oldAss);
    string toString();
#ifdef EXPANSIONEVAL
    int getExpansionCount();
    void setExpansionCount(int expansionCount);
#endif
private:
    PartialAssignment* calcNextBestPartialAssignment(IAssignment* oldAss);

protected:
    // Plan to build an assignment for
    TeamManager* tm;
    TeamObserver* to;
    list<Plan*> planList;
    shared_ptr<vector<const supplementary::AgentID*>> robots;
    vector<EntryPoint*> entryPointVector;
    // Fringe of the search tree
    vector<PartialAssignment*> fringe;
    bool addAlreadyAssignedRobots(PartialAssignment* pa,
            map<const supplementary::AgentID*, shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>*
                    simplePlanTreeMap);

#ifdef EXPANSIONEVAL
    int expansionCount;
#endif
};

} /* namespace alica */
