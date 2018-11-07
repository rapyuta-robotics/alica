/*
 * ExitPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef TERMINALSTATE_H_
#define TERMINALSTATE_H_

#include "State.h"

namespace alica
{
class PostCondition;
class ModelFactory;
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
    void setPostCondition(PostCondition* posCondition);
    PostCondition* _postCondition;
};

} // namespace alica

#endif /* TERMINALSTATE_H_ */
