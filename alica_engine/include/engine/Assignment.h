#pragma once

#include "IAssignment.h"
#include <supplementary/AgentID.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <sstream>

namespace alica {

class Plan;
class StateCollection;
class SuccessCollection;
class AssignmentCollection;
class EntryPoint;
class PartialAssignment;
class State;
struct AllocationAuthorityInfo;

/**
 * Contains all allocation information for a single plan. This includes the robot-task mapping, robot-state mapping and
 * success information.
 */
class Assignment : public IAssignment {
public:
    Assignment(PartialAssignment* pa);
    Assignment(Plan* p, shared_ptr<AllocationAuthorityInfo> aai);
    Assignment(Plan* p);
    virtual ~Assignment();
    Plan* getPlan();
    void setPlan(Plan* plan);
    StateCollection* getRobotStateMapping();
    std::shared_ptr<std::vector<const supplementary::AgentID*>> getAllRobotsSorted();
    AssignmentCollection* getEpRobotsMapping();
    std::shared_ptr<std::vector<const supplementary::AgentID*>> getRobotsWorking(long epid);
    std::shared_ptr<std::vector<const supplementary::AgentID*>> getRobotsWorkingSorted(EntryPoint* ep);
    std::shared_ptr<std::vector<const supplementary::AgentID*>> getRobotsWorking(EntryPoint* ep);
    int totalRobotCount();
    // shared_ptr<vector<EntryPoint*> > getEntryPoints();
    short getEntryPointCount();
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(EntryPoint* ep);
    std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(long epid);
    std::shared_ptr<SuccessCollection> getEpSuccessMapping();
    void setAllToInitialState(const std::vector<const supplementary::AgentID*>& robotIds, EntryPoint* defep);
    bool removeRobot(const supplementary::AgentID* robotId);
    void addRobot(const supplementary::AgentID* robotId, EntryPoint* e, State* s);
    bool isValid();
    bool isSuccessfull();
    bool isEqual(Assignment* otherAssignment);
    bool isEntryPointNonEmpty(EntryPoint* ep);
    bool updateRobot(const supplementary::AgentID* robotId, EntryPoint* ep, State* s);
    bool updateRobot(const supplementary::AgentID* robotId, EntryPoint* ep);
    bool removeRobot(const supplementary::AgentID* robotId, EntryPoint* ep);
    string assignmentCollectionToString();
    void addRobot(const supplementary::AgentID* robotId, EntryPoint* e);
    void moveRobots(State* from, State* to);
    EntryPoint* getEntryPointOfRobot(const supplementary::AgentID* robotId);
    shared_ptr<vector<const supplementary::AgentID*>> getAllRobots();
    void clear();
    std::string toString();
    std::string toHackString();

protected:
    /**
     * The Plan this Assignment refers to
     */
    Plan* plan;
    /**
     * The robot-to-state mapping of this assignment.
     */
    StateCollection* robotStateMapping;
    /**
     * Information about succeeded tasks.
     */
    shared_ptr<SuccessCollection> epSucMapping;
    AssignmentCollection* epRobotsMapping;
};
} /* namespace alica */
