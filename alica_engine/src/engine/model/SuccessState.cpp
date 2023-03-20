#include "engine/model/SuccessState.h"
#include "engine/model/Transition.h"
#include <sstream>

namespace alica
{

/**
 * Basic constructor
 */
SuccessState::SuccessState()
        : TerminalState(SUCCESS)
{
}

std::string SuccessState::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#SuccessState: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t Result:" << std::endl;
    ss << indent << "\t InTransitions: " << getInTransitions().size() << std::endl;
    for (const Transition* t : getInTransitions()) {
        ss << indent << "\t" << t->getId() << " " << t->getName() << std::endl;
    }
    ss << "#SuccessState" << std::endl;
    return ss.str();
}

} // namespace alica
