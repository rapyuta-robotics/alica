#pragma once

#include "engine/Types.h"
#include "supplementary/AgentID.h"


#include <list>
#include <memory>
#include <string>


namespace alica {

using std::list;
using std::shared_ptr;
using std::string;
using std::vector;

class EntryPoint;
class SuccessCollection;
class AssignmentCollection;

/**
 *  An IAssignment describes a potentially partial assignment of robots to EntryPoints within a plan.
 */
class IAssignment {
public:
    virtual ~IAssignment() {}
    virtual const std::vector<const supplementary::AgentID*>* getRobotsWorking(const EntryPoint* ep) const = 0;
    virtual const std::vector<const supplementary::AgentID*>* getRobotsWorking(int64_t epid) const = 0;
    virtual int totalRobotCount() = 0;
    /**
     * The shared_ptr of a vector of EntryPoints relevant to this assignment.
     */
    // virtual shared_ptr<vector<EntryPoint*> > getEntryPoints() = 0;
    /**
     * The number of distinct entrypoints
     * @param An int
     */
    virtual short getEntryPointCount() const = 0;
    /**
     * Returns all robot Ids working on the Task defined by ep and those which successfully completed it.
     * Behaviour is undefined if ep is not relevant or null.
     * @param ep The EntryPoint queried
     * @return A shared_ptr of a list of int of robot ids
     */
    virtual shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) = 0;
    /**
     * Similar to GetRobotsWorkingAndFinished, with duplicates removed.
     * Behaviour is undefined if ep is not relevant or null.
     * @param ep The EntryPoint queried
     * @return A shared_ptr of a list of int of robot ids
     */
    virtual shared_ptr<list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) = 0;
    /**
     * Returns all robot Ids working on the Task defined by ep
     * Behaviour is undefined if ep is not relevant or null.
     * @param ep The EntryPoint queried
     * @return A shared_ptr of a list of int of robot ids
     */
    virtual shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) = 0;
    /**
     * Information about succeeded tasks.
     */
    virtual shared_ptr<SuccessCollection> getEpSuccessMapping() = 0;
    /**
     * Checks whether the current assignment is valid
     */
    virtual bool isValid() const = 0;
    /**
     * Print AssignmentCollection
     */
    virtual string assignmentCollectionToString() = 0;

    virtual string toString() = 0;

    virtual AssignmentCollection* getEpRobotsMapping() = 0;

    double getMax() const { return max; }

    virtual void setMax(double max) { this->max = max; }

    double getMin() const { return min; }

    void setMin(double min) { this->min = min; }

    int getNumUnAssignedRobotIds() const { return unassignedRobotIds.size(); }

    const AgentSet& getUnassignedRobotIds() const { return unassignedRobotIds; }

protected:
    /**
     * The Ids of all robots available but not yet assigned.
     */
    AgentSet unassignedRobotIds;
    /**
     * The minimal utility this assignment can achieve.
     */
    double min;
    /**
     * the maximal utility this assignment can achieve.
     */
    double max;
};

} /* namespace alica */
