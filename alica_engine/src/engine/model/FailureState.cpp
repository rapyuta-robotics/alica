#include "engine/model/FailureState.h"
#include "engine/model/Transition.h"

namespace alica
{

FailureState::FailureState()
        : TerminalState(FAILURE)
{
}

FailureState::~FailureState() {}

std::string FailureState::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#FailureState: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t Result:" << std::endl;
    ss << indent << "\t InTransitions: " << getInTransitions().size() << std::endl;
    if (getInTransitions().size() != 0) {
        for (const Transition* t : getInTransitions()) {
            ss << indent << "\t" << t->getId() << " " << t->getName() << std::endl;
        }
    }
    ss << "#EndFailureState" << std::endl;
    return ss.str();
}

} // namespace alica
