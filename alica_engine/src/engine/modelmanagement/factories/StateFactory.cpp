#include "engine/modelmanagement/factories/StateFactory.h"
#include "engine/model/State.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"

namespace alica
{
State* StateFactory::create(const YAML::Node& stateNode)
{
    State* state = new State();
    Factory::setAttributes(stateNode, state);
    Factory::storeElement(state, alica::Strings::state);
    state->_inPlan = (Plan*) Factory::getElement(Factory::getReferencedId(stateNode[alica::Strings::parentPlan]));
    if (stateNode[alica::Strings::inTransitions]) {
        const YAML::Node& inTransitions = stateNode[alica::Strings::inTransitions];
        for (YAML::const_iterator it = inTransitions.begin(); it != inTransitions.end(); ++it) {
            Factory::stateInTransitionReferences.push_back(std::pair<int64_t, int64_t>(state->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (stateNode[alica::Strings::outTransitions]) {
        const YAML::Node& outTransitions = stateNode[alica::Strings::outTransitions];
        for (YAML::const_iterator it = outTransitions.begin(); it != outTransitions.end(); ++it) {
            Factory::stateOutTransitionReferences.push_back(std::pair<int64_t, int64_t>(state->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (stateNode[alica::Strings::abstractPlans]) {
        const YAML::Node& abstractPlans = stateNode[alica::Strings::abstractPlans];
        for (YAML::const_iterator it = abstractPlans.begin(); it != abstractPlans.end(); ++it) {
            Factory::statePlanReferences.push_back(std::pair<int64_t, int64_t>(state->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (stateNode[alica::Strings::variableBindings]) {
        const YAML::Node& variableBindings = stateNode[alica::Strings::variableBindings];
        for (YAML::const_iterator it = variableBindings.begin(); it != variableBindings.end(); ++it) {
            //state->_variableBindingGrp.push_back(VariableBindingFactory::create(*it));
        }
    }
    return state;
}
} // namespace alica
