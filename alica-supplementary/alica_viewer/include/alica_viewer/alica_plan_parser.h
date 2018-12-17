#pragma once

#include <engine/AgentIDConstPtr.h>
#include <engine/PlanRepository.h>
#include <engine/Types.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/parser/PlanParser.h>
#include <essentials/AgentIDManager.h>
#include <unordered_map>

namespace alica
{

class PlanTree;
class AgentInfo;

using AgentInfoMap = std::unordered_map<AgentIDConstPtr, AgentInfo, AgentIDHash>;
using PlanTreeMap = std::unordered_map<AgentIDConstPtr, std::unique_ptr<PlanTree>, AgentIDHash>;
using PlanTreeVectorMap = std::unordered_map<int64_t, std::vector<std::unique_ptr<PlanTree>>>;

class AgentInfo
{
  public:
    AgentInfo() {}

    AgentInfo(AgentIDConstPtr idarg, const std::string& tmp_name)
        : id(idarg)
        , name(tmp_name)
    {
    }
    AgentIDConstPtr id;
    std::string name;
};

class PlanTree
{
  public:
    PlanTree(AlicaTime creationTime);
    PlanTree(const PlanTree& other, const PlanTree& parent);
    bool operator==(const PlanTree& other) const;
    void setParent(const PlanTree& parent);
    bool setState(const State* state);
    void addChildren(std::unique_ptr<PlanTree> child);
    void addRobot(AgentIDConstPtr robotId);
    void mergeRobots(const AgentGrp& robotIds);
    /** Merge the src plan tree as a branch of current tree */
    void mergePlanTree(const PlanTree& src);

    bool isValid() const { return _state != nullptr && _entryPoint != nullptr; }
    bool parentExists() const { return _parent != nullptr; }
    const State* getState() const { return _state; }
    const EntryPoint* getEntryPoint() const { return _entryPoint; }
    const AgentGrp& getRobots() const { return _robotIds; }
    void getRobotsSorted(AgentGrp& robotIds) const;
    const PlanTreeVectorMap& getChildren() const { return _children; }
    int getX() const { return _x; }
    int getY() const { return _y; }
    AlicaTime getCreationTime() const { return _time; }

  private:
    const PlanTree* _parent;
    const State* _state;
    const EntryPoint* _entryPoint;
    AgentGrp _robotIds;
    PlanTreeVectorMap _children;
    AlicaTime _time;
    int _numOfChildren;
    int _x;
    int _y;
};

class AlicaPlan
{
  public:
    AlicaPlan(int argc, char* argv[]);
    void combinePlanTree(PlanTree& planTree) const;
    void handlePlanTreeInfo(const PlanTreeInfo& incoming);
    /**
     * Constructs a PlanTree from a received message
     * @param robotId The id of the robot.
     * @param ids The list of long encoding of a robot's plantree as received in a PlanTreeInfo message.
     * @return std::unique_ptr<PlanTree>
     */
    std::unique_ptr<PlanTree> planTreeFromMessage(AgentIDConstPtr robotId, const IdGrp& ids);

    const PlanTreeMap& getPlanTrees() const { return _planTrees; }
    const AgentInfoMap& getAgentInfoMap() const { return _agentInfos; }
    const AgentInfo* getAgentInfo(AgentIDConstPtr agentID) const;

  private:
    PlanRepository _planRepository;
    PlanParser _planParser;
    PlanTreeMap _planTrees;
    PlanTree* _combinedPlanTree;
    AgentInfoMap _agentInfos;
    AlicaClock _clock;
    essentials::AgentIDManager _agentIDManager;
    AlicaTime _teamTimeOut;
};

} // namespace alica
