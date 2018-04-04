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
        : State() 
        , _terminal(true)
        , _postCondition(nullptr) {}

TerminalState::~TerminalState() {
}

void TerminalState::setPostCondition(PostCondition* posCondition) {
    _postCondition = posCondition;
}

}  // namespace alica
