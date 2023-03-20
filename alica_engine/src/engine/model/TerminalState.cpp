#include "engine/model/TerminalState.h"
#include "engine/model/PostCondition.h"

namespace alica
{

TerminalState::TerminalState(StateType t)
        : State(t)
        , _postCondition(nullptr)
{
}

void TerminalState::setPostCondition(PostCondition* posCondition)
{
    _postCondition = posCondition;
}

} // namespace alica
