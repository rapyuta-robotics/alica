#pragma once
#include "State.h"

namespace alica
{
class PostCondition;
/**
 * A terminal state within a plan. Indicates termination of the corresponding task
 */
class TerminalState : public State
{
public:
    TerminalState(StateType t);
    void setPostCondition(PostCondition* posCondition);
    const PostCondition* getPostCondition() const { return _postCondition; }

protected:
    PostCondition* _postCondition;
};

} // namespace alica
