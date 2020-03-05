#include "engine/modelmanagement/factories/TerminalStateFactory.h"
#include "engine/model/FailureState.h"
#include "engine/model/State.h"
#include "engine/model/SuccessState.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"

namespace alica
{
TerminalState* TerminalStateFactory::create(const YAML::Node& terminalStateNode, AbstractPlan* plan)
{
    TerminalState* terminalState = nullptr;
    std::string stateType = Factory::getValue<std::string>(terminalStateNode, alica::Strings::stateType);
    if (getValue<bool>(terminalStateNode, alica::Strings::isSuccessState)) {
        terminalState = new SuccessState();
        terminalState->_type = State::StateType::SUCCESS;
    } else {
        terminalState = new FailureState();
        terminalState->_type = State::StateType::FAILURE;
    }
    Factory::setAttributes(terminalStateNode, terminalState);
    Factory::storeElement(terminalState, alica::Strings::state);
    terminalState->_inPlan = (Plan*) Factory::getElement(Factory::getReferencedId(terminalStateNode[alica::Strings::parentPlan]));
    if (Factory::isValid(terminalStateNode[alica::Strings::postCondition])) {
        terminalState->_postCondition = PostConditionFactory::create(terminalStateNode[alica::Strings::postCondition], plan);
    }

    if (Factory::isValid(terminalStateNode[alica::Strings::inTransitions])) {
        const YAML::Node& inTransitions = terminalStateNode[alica::Strings::inTransitions];
        for (YAML::const_iterator it = inTransitions.begin(); it != inTransitions.end(); ++it) {
            Factory::stateInTransitionReferences.push_back(std::pair<int64_t, int64_t>(terminalState->getId(), Factory::getReferencedId(*it)));
        }
    }
    return terminalState;
}
} // namespace alica
