#include <engine/AlicaClock.h>
#include <engine/Types.h>

#include <ostream>

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
    const EntryPoint* getEntryPoint() const { return _entryPoint; }
    void setEntryPoint(const EntryPoint* entryPoint);
    const State* getState() const { return _state; }
    void setState(const State* state);
    const std::vector<std::unique_ptr<SimplePlanTree>>& getChildren() const { return _children; }
    std::vector<std::unique_ptr<SimplePlanTree>>& editChildren() { return _children; }
    SimplePlanTree* getParent() const { return _parent; }
    void setParent(SimplePlanTree* p) { _parent = p; }
    alica::AgentId getAgentId() const { return _agentId; }
    void setAgentId(alica::AgentId agentId) { _agentId = agentId; }
    bool isNewSimplePlanTree() const { return _isNew; }
    void setProcessed() { _isNew = false; }
    AlicaTime getReceiveTime() const { return _receiveTime; }
    void setReceiveTime(AlicaTime receiveTime);
    const IdGrp& getStateIds() const { return _stateIds; }
    void setStateIds(const IdGrp& stateIds);
    bool containsPlan(const AbstractPlan* plan) const;

private:
    friend std::ostream& operator<<(std::ostream& out, const SimplePlanTree& spt);
    /**
     * The parent SimplePlanTree
     */
    SimplePlanTree* _parent;
    std::vector<std::unique_ptr<SimplePlanTree>> _children;
    /**
     * The state occupied by the respective robot.
     */
    const State* _state;
    const EntryPoint* _entryPoint;
    /**
     * The id of the robot to which this tree refers to
     */
    alica::AgentId _agentId;
    bool _isNew;
    /**
     * The timestamp denoting when this tree was received.
     */
    AlicaTime _receiveTime;
    IdGrp _stateIds;
};
std::ostream& operator<<(std::ostream& out, const SimplePlanTree& spt);
} /* namespace alica */
