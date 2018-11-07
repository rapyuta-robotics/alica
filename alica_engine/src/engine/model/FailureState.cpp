/*
 * FailurePoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/FailureState.h"
#include "engine/model/Transition.h"

namespace alica
{

FailureState::FailureState()
        : TerminalState(FAILURE)
{
}

FailureState::~FailureState() {}

std::string FailureState::toString() const
{
    std::stringstream ss;
    ss << "#FailureState: " << getName() << " " << getId() << std::endl;
    ss << "\t Result:" << std::endl;
    ss << "\t InTransitions: " << getInTransitions().size() << std::endl;
    if (getInTransitions().size() != 0) {
        for (const Transition* t : getInTransitions()) {
            ss << "\t" << t->getId() << " " << t->getName() << std::endl;
        }
    }
    ss << "#EndFailureState" << std::endl;
    return ss.str();
}

} // namespace alica
