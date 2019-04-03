#include "engine/modelmanagement/factories/TransitionFactory.h"
#include "engine/model/Transition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"

namespace alica
{
    Transition* TransitionFactory::create(const YAML::Node& transitionNode, Plan* plan)
    {
        Transition* transition = new Transition();
        Factory::setAttributes(transitionNode, transition);
        Factory::storeElement(transition, alica::Strings::transition);
        if (Factory::isValid(transitionNode[alica::Strings::inState])) {
            Factory::transitionInStateReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::inState])));
        }
        if (Factory::isValid(transitionNode[alica::Strings::outState])) {
            Factory::transitionOutStateReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::outState])));
        }
        if (Factory::isValid(transitionNode[alica::Strings::synchronisation])) {
            Factory::transitionSynchReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::synchronisation])));
        }
        if (Factory::isValid(transitionNode[alica::Strings::preCondition])) {
            transition->_preCondition = PreConditionFactory::create(transitionNode[alica::Strings::preCondition], plan);
        }

        return transition;
    }

    void TransitionFactory::attachReferences() {
        // transitionOutStateReferences
        for (std::pair<int64_t, int64_t> pairs : Factory::transitionOutStateReferences) {
            Transition* t = (Transition*) Factory::getElement(pairs.first);
            State* st = (State*)Factory::getElement(pairs.second);
            t->_outState = st;
        }
        Factory::transitionOutStateReferences.clear();
        // transitionInStateReferences
        for (std::pair<int64_t, int64_t> pairs : Factory::transitionInStateReferences) {
            Transition* t = (Transition*) Factory::getElement(pairs.first);
            State* st = (State*)Factory::getElement(pairs.second);
            t->_inState = st;
        }
        Factory::transitionInStateReferences.clear();
        // transitionSynchReferences
        for (std::pair<int64_t, int64_t> pairs : Factory::transitionSynchReferences) {
            Transition* t = (Transition*) Factory::getElement(pairs.first);
            Synchronisation* sync = (Synchronisation*) Factory::getElement(pairs.second);
            t->setSynchronisation(sync);
        }
        Factory::transitionSynchReferences.clear();
    }
} // namespace alica
