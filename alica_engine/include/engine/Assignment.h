#pragma once

#include "IAssignment.h"
#include "engine/Types.h"
#include "engine/collections/StateCollection.h"
#include <supplementary/AgentID.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

namespace alica
{

class Plan;
class AssignmentCollection;
class SuccessCollection;
class EntryPoint;
class PartialAssignment;
class State;
struct AllocationAuthorityInfo;

/**
 * Contains all allocation information for a single plan. This includes the robot-task mapping, robot-state mapping and
 * success information.
 */
class Assignment final : public IAssignment
{
  public:
    Assignment(PartialAssignment* pa);
    Assignment(const Plan* p, shared_ptr<AllocationAuthorityInfo> aai);
    Assignment(const Plan* p);
    virtual ~Assignment();
    const Plan* getPlan() const { return plan; }
    void setPlan(const Plan* plan);
    StateCollection* getRobotStateMapping();

    virtual AssignmentCollection* getEpRobotsMapping() const override { return epRobotsMapping; }
    void getAllRobots(AgentGrp& o_robots);
    void getAllRobotsSorted(AgentGrp& o_robots);
    const AgentGrp* getRobotsWorking(int64_t epid) const override;
    void getRobotsWorkingSorted(const EntryPoint* ep, AgentGrp& o_robots);
    const AgentGrp* getRobotsWorking(const EntryPoint* ep) const override;
    int totalRobotCount() const override;
    bool hasRobot(AgentIDConstPtr id) const
    {
        return std::find(robotStateMapping->getRobots().begin(), robotStateMapping->getRobots().end(), id) != robotStateMapping->getRobots().end();
    }
    const AgentGrp& getAllRobots() const { return robotStateMapping->getRobots(); }

    short getEntryPointCount() const override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) override;
    virtual std::shared_ptr<SuccessCollection> getEpSuccessMapping() const override;
    void setAllToInitialState(const AgentGrp& robotIds, const EntryPoint* defep);
    bool removeRobot(const supplementary::AgentID* robotId);
    void addRobot(const supplementary::AgentID* robotId, const EntryPoint* e, const State* s);
    bool isValid() const override;
    bool isSuccessfull() const;
    bool isEqual(Assignment* otherAssignment);
    bool isEntryPointNonEmpty(const EntryPoint* ep) const;
    bool updateRobot(const supplementary::AgentID* robotId, const EntryPoint* ep, const State* s);
    bool updateRobot(const supplementary::AgentID* robotId, const EntryPoint* ep);
    bool removeRobot(const supplementary::AgentID* robotId, const EntryPoint* ep);
    virtual std::string assignmentCollectionToString() const override;
    void addRobot(const supplementary::AgentID* robotId, const EntryPoint* e);
    void moveRobots(const State* from, const State* to);
    const EntryPoint* getEntryPointOfRobot(const supplementary::AgentID* robotId);

    void clear();
    virtual std::string toString() const override;
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
