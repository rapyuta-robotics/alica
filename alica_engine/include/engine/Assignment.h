#pragma once


#include <vector>
#include <memory>
#include <algorithm>
#include <sstream>

#include "IAssignment.h"
#include <supplementary/AgentID.h>
#include "engine/Types.h"


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
class Assignment final : public IAssignment {
public:
    Assignment(PartialAssignment* pa);
    Assignment(const Plan* p, shared_ptr<AllocationAuthorityInfo> aai);
    Assignment(const Plan* p);
    virtual ~Assignment();
    const Plan* getPlan() const {return plan;}
    void setPlan(const Plan* plan);
    StateCollection* getRobotStateMapping();
    
    AssignmentCollection* getEpRobotsMapping();
    void getAllRobots(AgentSet& o_robots);
    void getAllRobotsSorted(AgentSet& o_robots);
    const AgentSet* getRobotsWorking(int64_t epid) const override;
    void getRobotsWorkingSorted(const EntryPoint* ep, AgentSet& o_robots);
    const AgentSet* getRobotsWorking(const EntryPoint* ep) const override;
    int totalRobotCount();
    
    short getEntryPointCount() const override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) override;
    std::shared_ptr<SuccessCollection> getEpSuccessMapping() override;
    void setAllToInitialState(const AgentSet& robotIds, const EntryPoint* defep);
    bool removeRobot(const supplementary::AgentID* robotId);
    void addRobot(const supplementary::AgentID* robotId, const EntryPoint* e, const State* s);
    bool isValid() const override;
    bool isSuccessfull() const;
    bool isEqual(Assignment* otherAssignment);
    bool isEntryPointNonEmpty(const EntryPoint* ep) const;
    bool updateRobot(const supplementary::AgentID* robotId, const EntryPoint* ep, const State* s);
    bool updateRobot(const supplementary::AgentID* robotId, const EntryPoint* ep);
    bool removeRobot(const supplementary::AgentID* robotId, const EntryPoint* ep);
    std::string assignmentCollectionToString();
    void addRobot(const supplementary::AgentID* robotId, const EntryPoint* e);
    void moveRobots(const State* from,const State* to);
    const EntryPoint* getEntryPointOfRobot(const supplementary::AgentID* robotId);
    
    void clear();
    std::string toString();
    std::string toHackString();

protected:
    /**
     * The Plan this Assignment refers to
     */
    const Plan* plan;
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
