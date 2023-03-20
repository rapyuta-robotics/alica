#include "engine/model/FailureState.h"
#include "engine/model/Transition.h"
#include <sstream>

namespace alica
{

FailureState::FailureState()
        : TerminalState(FAILURE)
{
}

std::string FailureState::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#FailureState: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t Result:" << std::endl;
    ss << indent << "\t InTransitions: " << getInTransitions().size() << std::endl;
    for (const Transition* t : getInTransitions()) {
        ss << indent << "\t" << t->getId() << " " << t->getName() << std::endl;
    }
    ss << "#EndFailureState" << std::endl;
    return ss.str();
}

} // namespace alica
