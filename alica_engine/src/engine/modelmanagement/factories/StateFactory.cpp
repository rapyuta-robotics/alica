#include "engine/modelmanagement/factories/StateFactory.h"
#include "engine/model/State.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"
#include "engine/modelmanagement/factories/ConfAbstractPlanWrapperFactory.h"

namespace alica
{
State* StateFactory::create(const YAML::Node& stateNode)
{
    State* state = new State();
    state->_type = State::StateType::NORMAL;
    Factory::setAttributes(stateNode, state);
    Factory::storeElement(state, alica::Strings::state);
    state->_inPlan = (Plan*) Factory::getElement(Factory::getReferencedId(stateNode[alica::Strings::parentPlan]));

    if (Factory::isValid(stateNode[alica::Strings::inTransitions])) {
        const YAML::Node& inTransitions = stateNode[alica::Strings::inTransitions];
        for (YAML::const_iterator it = inTransitions.begin(); it != inTransitions.end(); ++it) {
            Factory::stateInTransitionReferences.push_back(std::pair<int64_t, int64_t>(state->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (Factory::isValid(stateNode[alica::Strings::outTransitions])) {
        const YAML::Node& outTransitions = stateNode[alica::Strings::outTransitions];
        for (YAML::const_iterator it = outTransitions.begin(); it != outTransitions.end(); ++it) {
            Factory::stateOutTransitionReferences.push_back(std::pair<int64_t, int64_t>(state->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (Factory::isValid(stateNode[alica::Strings::confAbstractPlanWrappers])) {
        const YAML::Node& confAbstractPlanWrappers = stateNode[alica::Strings::confAbstractPlanWrappers];
        for (YAML::const_iterator it = confAbstractPlanWrappers.begin(); it != confAbstractPlanWrappers.end(); ++it) {
            state->_confAbstractPlanWrappers.push_back(ConfAbstractPlanWrapperFactory::create(*it));
        }
    }
    if (Factory::isValid(stateNode[alica::Strings::variableBindings])) {
        const YAML::Node& variableBindings = stateNode[alica::Strings::variableBindings];
        for (YAML::const_iterator it = variableBindings.begin(); it != variableBindings.end(); ++it) {
            state->_variableBindingGrp.push_back(VariableBindingFactory::create(*it));
        }
    }
    return state;
}

void StateFactory::attachReferences()
{
    VariableBindingFactory::attachReferences();
    ConfAbstractPlanWrapperFactory::attachReferences();

    // stateInTransitionReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::stateInTransitionReferences) {
        Transition* t = (Transition*) Factory::getElement(pairs.second);
        State* st = (State*) Factory::getElement(pairs.first);
        st->_inTransitions.push_back(t);
    }
    Factory::stateInTransitionReferences.clear();

    // stateOutTransitionReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::stateOutTransitionReferences) {
        State* st = (State*) Factory::getElement(pairs.first);
        Transition* t = (Transition*) Factory::getElement(pairs.second);
        st->_outTransitions.push_back(t);
    }
    Factory::stateOutTransitionReferences.clear();
}
} // namespace alica
