#include "supplementary/AgentID.h"

#include <unordered_set>
#include <list>
#include <memory>

namespace alica {

class State;
class EntryPoint;
class AbstractPlan;

/**
 * A SimplePlanTree is a simplified version of the RunningPlan, usually created from an incoming message. It thus
 * represents the plan graph of another robot.
 */
class SimplePlanTree {
public:
    SimplePlanTree();
    virtual ~SimplePlanTree();
    const EntryPoint* getEntryPoint() const { return entryPoint; }
    void setEntryPoint(const EntryPoint* entryPoint);
    const State* getState() const { return state; }
    void setState(const State* state);
    const std::unordered_set<std::shared_ptr<SimplePlanTree>>& getChildren() const;
    std::unordered_set<std::shared_ptr<SimplePlanTree>>& editChildren() { return children; }

    void setChildren(const std::unordered_set<std::shared_ptr<SimplePlanTree>>& children);
    const supplementary::AgentID* getRobotId();
    void setRobotId(const supplementary::AgentID* robotId);
    bool isNewSimplePlanTree() const;
    void setNewSimplePlanTree(bool newSimplePlanTree);
    long getReceiveTime() const;
    void setReceiveTime(long receiveTime);
    const std::list<int64_t>& getStateIds() const;
    void setStateIds(const std::list<int64_t>& stateIds);
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
    const supplementary::AgentID* robotId;
    bool newSimplePlanTree;
    /**
     * The timestamp denoting when this tree was received.
     */
    long receiveTime;
    std::list<long> stateIds;
};

} /* namespace alica */
