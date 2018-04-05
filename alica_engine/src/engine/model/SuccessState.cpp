/*
 * SuccessState.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/SuccessState.h"
#include "engine/model/Transition.h"
#include <sstream>

namespace alica {

/**
 * Basic constructor
 */
SuccessState::SuccessState()
    : TerminalState(Success)
{}

SuccessState::~SuccessState() {}

std::string SuccessState::toString() const {
    std::stringstream ss;
    ss << "#SuccessState: " << getName() << " " << getId() << std::endl;
    ss << "\t Result:" << std::endl;
    ss << "\t InTransitions: " << getInTransitions().size() << std::endl;
    if (getInTransitions().size() != 0) {
        for (const Transition* t : getInTransitions()) {
            ss << "\t" << t->getId() << " " << t->getName() << std::endl;
        }
    }
    ss << "#SuccessState" << std::endl;
    return ss.str();
}

}  // namespace alica
