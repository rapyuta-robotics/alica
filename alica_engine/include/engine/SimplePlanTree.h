#include <engine/AlicaClock.h>
#include <engine/Types.h>

#include <supplementary/AgentID.h>

#include <list>
#include <memory>
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
    const EntryPoint* getEntryPoint() const { return entryPoint; }
    void setEntryPoint(const EntryPoint* entryPoint);
    const State* getState() const { return state; }
    void setState(const State* state);
    const std::unordered_set<std::shared_ptr<SimplePlanTree>>& getChildren() const;
    std::unordered_set<std::shared_ptr<SimplePlanTree>>& editChildren() { return children; }

    void setChildren(const std::unordered_set<std::shared_ptr<SimplePlanTree>>& children);
    AgentIDConstPtr getRobotId() { return robotId; }
    void setRobotId(AgentIDConstPtr robotId);
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
    SimplePlanTree* parent;
    std::unordered_set<std::shared_ptr<SimplePlanTree>> children;
    /**
     * The state occupied by the respective robot.
     */
    const State* state;
    const EntryPoint* entryPoint;
    /**
     * The id of the robot to which this tree refers to
     */
    AgentIDConstPtr robotId;
    bool newSimplePlanTree;
    /**
     * The timestamp denoting when this tree was received.
     */
    AlicaTime receiveTime;
    IdGrp stateIds;
};

} /* namespace alica */
