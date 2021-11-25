#include <engine/AlicaClock.h>
#include <engine/Types.h>

#include <ostream>
#include <unordered_set>

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
    // Call after the parent, entry point & state is set
    void computeContextHash();
    const std::vector<std::unique_ptr<SimplePlanTree>>& getChildren() const { return _children; }
    std::vector<std::unique_ptr<SimplePlanTree>>& editChildren() { return _children; }
    SimplePlanTree* getParent() const { return _parent; }
    void setParent(SimplePlanTree* p) { _parent = p; }
    AgentId getAgentId() const { return _agentId; }
    void setAgentId(AgentId agentId) { _agentId = agentId; }
    bool isNewSimplePlanTree() const { return _isNew; }
    void setProcessed() { _isNew = false; }
    AlicaTime getReceiveTime() const { return _receiveTime; }
    void setReceiveTime(AlicaTime receiveTime);
    const IdGrp& getDynamicStateIDPairs() const { return _dynamicStateIdPairs; }
    void setDynamicStateIDPairs(const IdGrp& dynamicStateIdPairs);
    bool containsContext(std::size_t parentContextHash, const AbstractPlan* plan) const;

private:
    bool containsContexts(const std::unordered_set<std::size_t>& contextHashes) const;

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
    AgentId _agentId;
    bool _isNew;
    /**
     * The timestamp denoting when this tree was received.
     */
    AlicaTime _receiveTime;
    IdGrp _dynamicStateIdPairs;

    std::size_t _contextHash;
};
std::ostream& operator<<(std::ostream& out, const SimplePlanTree& spt);
} /* namespace alica */
