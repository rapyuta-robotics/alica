/*
 * ExitPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TerminalState.h"
#include "engine/model/PostCondition.h"

namespace alica {

TerminalState::TerminalState()
        : State() {
    this->terminal = true;
    this->postCondition = nullptr;
}

TerminalState::~TerminalState() {
    // TODO Auto-generated destructor stub
}

PostCondition* TerminalState::getPostCondition() {
    return postCondition;
}

void TerminalState::setPostCondition(PostCondition* posCondition) {
    this->postCondition = posCondition;
}

}  // namespace alica
