#include <engine/AlicaClock.h>
#include <engine/Types.h>

#include <engine/AgentIDConstPtr.h>

#include <vector>

namespace alica
{

/**
 * A SimplePlanTree is a simplified version of the RunningPlan, usually created from an incoming message. It thus
 * represents the plan graph of another robot.
 */
class SimplePlanTree
{
public:
    SimplePlanTree();
    ~SimplePlanTree();
    const EntryPoint* getEntryPoint() const { return entryPoint; }
    void setEntryPoint(const EntryPoint* entryPoint);
    const State* getState() const { return state; }
    void setState(const State* state);
    const std::vector<std::unique_ptr<SimplePlanTree>>& getChildren() const { return _children; }
    std::vector<std::unique_ptr<SimplePlanTree>>& editChildren() { return _children; }

    AgentIDConstPtr getAgentId() const { return _agentId; }
    void setAgentId(AgentIDConstPtr agentId) { _agentId = agentId; }
    bool isNewSimplePlanTree() const;
    void setNewSimplePlanTree(bool newSimplePlanTree);
    AlicaTime getReceiveTime() const;
    void setReceiveTime(AlicaTime receiveTime);
    const IdGrp& getStateIds() const { return stateIds; }
    void setStateIds(const IdGrp& stateIds);
    bool containsPlan(const AbstractPlan* plan) const;
    std::string toString() const;

protected:
    /**
     * The parent SimplePlanTree
     */
    SimplePlanTree* _parent;
    std::vector<std::unique_ptr<SimplePlanTree>> _children;
    /**
     * The state occupied by the respective robot.
     */
    const State* state;
    const EntryPoint* entryPoint;
    /**
     * The id of the robot to which this tree refers to
     */
    AgentIDConstPtr _agentId;
    bool newSimplePlanTree;
    /**
     * The timestamp denoting when this tree was received.
     */
    AlicaTime receiveTime;
    IdGrp stateIds;
};

} /* namespace alica */
