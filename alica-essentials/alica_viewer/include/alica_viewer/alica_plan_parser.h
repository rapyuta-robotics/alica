#pragma once

#include <engine/PlanRepository.h>
#include <engine/Types.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/parser/PlanParser.h>
#include <supplementary/AgentID.h>
#include <supplementary/AgentIDManager.h>
#include <unordered_map>

namespace alica
{

class PlanTree;
class AgentInfo;

using AgentInfoMap = std::unordered_map<const supplementary::AgentID*, AgentInfo, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>;
using PlanTreeMap =
    std::unordered_map<const supplementary::AgentID*, std::unique_ptr<PlanTree>, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>;
using PlanTreeVectorMap = std::unordered_map<int64_t, std::vector<std::unique_ptr<PlanTree>>>;

class AgentInfo
{
  public:
    AgentInfo() {}

    AgentInfo(int tmp_id, const std::string& tmp_name)
        : id(tmp_id)
        , name(tmp_name)
    {
    }

    int id;
    std::string name;
};

class PlanTree
{
  public:
    PlanTree();
    PlanTree(const PlanTree& other, const PlanTree& parent);
    bool operator==(const PlanTree& other) const;
    void setParent(const PlanTree& parent);
    bool setState(const State* state);
    void addChildren(std::unique_ptr<PlanTree> child);
    void addRobot(const supplementary::AgentID* robotId);
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

  private:
    const PlanTree* _parent;
    const State* _state;
    const EntryPoint* _entryPoint;
    AgentGrp _robotIds;
    PlanTreeVectorMap _children;
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
    std::unique_ptr<PlanTree> planTreeFromMessage(const supplementary::AgentID* robotId, const IdGrp& ids);

    const PlanTreeMap& getPlanTrees() const { return _planTrees; }
    const AgentInfoMap& getAgentInfoMap() const { return _agentInfos; }
    const AgentInfo* getAgentInfo(const supplementary::AgentID* agentID) const;

  private:
    PlanRepository _planRepository;
    PlanParser _planParser;
    PlanTreeMap _planTrees;
    PlanTree* _combinedPlanTree;
    AgentInfoMap _agentInfos;
    supplementary::AgentIDManager _agentIDManager;
};

} // namespace alica
