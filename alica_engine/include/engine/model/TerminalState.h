#pragma once
#include "State.h"

namespace alica
{
class PostCondition;
class ModelFactory;
class TerminalStateFactory;
/**
 * A terminal state within a plan. Indicates termination of the corresponding task
 */
class TerminalState : public State
{
public:
    TerminalState(StateType t);
    virtual ~TerminalState();
    const PostCondition* getPostCondition() const { return _postCondition; }

protected:
    friend ModelFactory;
    friend TerminalStateFactory;
    void setPostCondition(PostCondition* posCondition);
    PostCondition* _postCondition;
};

} // namespace alica
