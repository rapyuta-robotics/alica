/*
 * ExitPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef TERMINALSTATE_H_
#define TERMINALSTATE_H_

#include "State.h"

namespace alica {
class PostCondition;

/**
 * A terminal state within a plan. Indicates termination of the corresponding task
 */
class TerminalState : public State {
public:
    TerminalState();
    virtual ~TerminalState();
    PostCondition* getPostCondition();
    void setPostCondition(PostCondition* posCondition);

protected:
    PostCondition* postCondition;
};

}  // namespace alica

#endif /* TERMINALSTATE_H_ */
