#pragma once

#include "IAssignment.h"
#include "engine/Types.h"
#include <engine/model/Plan.h>
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

class AgentStatePairs
{
public:
    AgentStatePairs() {}
    bool hasAgent(const AgentIDConstPtr id) const;
    const State* getStateOf(const AgentIDConstPtr id) const;

    const std::vector<AgentStatePair>& getRaw() const { return _data; }
    std::vector<AgentStatePair>& editRaw() { return _data; }

    int size() const { return static_cast<int>(_data.size()); }

    void clear() { _data.clear(); }
    void emplace_back(AgentIDConstPtr id, const State* s) { _data.emplace_back(id, s); }

    void removeAt(int idx) { _data.erase(_data.begin() + idx); }
    void removeAllIn(const AgentGrp& agents);

    std::vector<AgentStatePair>::iterator begin() { return _data.begin(); }
    std::vector<AgentStatePair>::iterator end() { return _data.end(); }
    std::vector<AgentStatePair>::const_iterator begin() const { return _data.begin(); }
    std::vector<AgentStatePair>::const_iterator end() const { return _data.end(); }

private:
    std::vector<AgentStatePair> _data;
};

/**
 * Contains all allocation information for a single plan. This includes the robot-task mapping, robot-state mapping and
 * success information.
 */
class Assignment
{
public:
    Assignment();
    Assignment(const PartialAssignment& pa);
    Assignment(const Plan* p, const AllocationAuthorityInfo& aai);
    Assignment(const Plan* p);

    Assignment(const Assignment& o);
    Assignment& operator=(const Assignment& o);

    const Plan* getPlan() const { return _plan; }

    bool isValid() const;
    bool isSuccessfull() const;

    bool hasAgent(AgentIDConstPtr id) const;
    int getEntryPointCount() const { return static_cast<int>(_assignmentData.size()); }
    const EntryPoint* getEntryPoint(int idx) const { return _plan->getEntryPoints()[idx]; }
    const EntryPoint* getEntryPointOfAgent(AgentIDConstPtr id) const;

    void getAllAgents(AgentGrp& o_agents) const;
    const AgentStatePairs& getAgentsWorking(int idx) const { return _assignmentData[idx]; }
    const AgentStatePairs* getAgentsWorking(const EntryPoint* ep) const;
    void getAgentsWorking(const EntryPoint* ep, AgentGrp& o_agents) const;
    void getAgentsWorking(int idx, AgentGrp& o_agents) const;
    void getAgentsWorkingAndFinished(const EntryPoint* ep, AgentGrp& o_agents) const;
    double getLastUtilityValue() const { return _lastUtility; }

    void getAgentsInState(const State* s, AgentGrp& o_agents) const;

    bool updateAgent(AgentIDConstPtr agent, const EntryPoint* e);
    void setAllToInitialState(const AgentGrp& agents, const EntryPoint* e);
    void clear();
    void moveAllFromTo(const EntryPoint* scope, const State* from, const State* to);
    void adaptTaskChangesFrom(const Assignment& as);

private:
    friend std::ostream& operator<<(std::ostream& out, const Assignment& a);
    const Plan* _plan;
    std::vector<AgentStatePairs> _assignmentData;
    std::vector<AgentGrp> _successData;
    double _lastUtility;
};

std::ostream& operator<<(std::ostream& out, const Assignment& a);
/*
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
    std::string toHackString() const;

  protected:
    const Plan* plan;
    StateCollection* robotStateMapping;
    shared_ptr<SuccessCollection> epSucMapping;
    AssignmentCollection* epRobotsMapping;
};*/
} /* namespace alica */
