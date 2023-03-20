#include "engine/modelmanagement/factories/TransitionFactory.h"
#include "engine/model/Transition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/KeyMappingFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"

namespace alica
{
Transition* TransitionFactory::create(const YAML::Node& transitionNode, Plan* plan)
{
    std::unique_ptr<KeyMapping> keyMapping;
    if (Factory::isValid(transitionNode[alica::Strings::keyMapping])) {
        keyMapping = KeyMappingFactory::create(transitionNode[alica::Strings::keyMapping]);
    }

    auto* transition = new Transition(std::move(keyMapping));
    Factory::setAttributes(transitionNode, transition);
    Factory::storeElement(transition, alica::Strings::transition);

    if (Factory::isValid(transitionNode[alica::Strings::inState])) {
        Factory::transitionInStateReferences.push_back(
                std::pair<int64_t, int64_t>(transition->getId(), Factory::getReferencedId(transitionNode[alica::Strings::inState])));
    }
    if (Factory::isValid(transitionNode[alica::Strings::outState])) {
        Factory::transitionOutStateReferences.push_back(
                std::pair<int64_t, int64_t>(transition->getId(), Factory::getReferencedId(transitionNode[alica::Strings::outState])));
    }
    if (Factory::isValid(transitionNode[alica::Strings::synchronisation])) {
        Factory::transitionSynchReferences.push_back(
                std::pair<int64_t, int64_t>(transition->getId(), Factory::getReferencedId(transitionNode[alica::Strings::synchronisation])));
    }
    if (Factory::isValid(transitionNode[alica::Strings::condition])) {
        Factory::transitionConditionReferences.push_back(
                std::pair<int64_t, int64_t>(transition->getId(), Factory::getReferencedId(transitionNode[alica::Strings::condition])));
    }

    return transition;
}

void TransitionFactory::attachReferences()
{
    ConditionFactory::attachReferences();
    // transitionOutStateReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::transitionOutStateReferences) {
        Transition* t = (Transition*) Factory::getElement(pairs.first);
        State* st = (State*) Factory::getElement(pairs.second);
        t->setOutState(st);
    }
    Factory::transitionOutStateReferences.clear();
    // transitionInStateReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::transitionInStateReferences) {
        Transition* t = (Transition*) Factory::getElement(pairs.first);
        State* st = (State*) Factory::getElement(pairs.second);
        t->setInState(st);
    }
    Factory::transitionInStateReferences.clear();
    // transitionSynchReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::transitionSynchReferences) {
        Transition* t = (Transition*) Factory::getElement(pairs.first);
        Synchronisation* sync = (Synchronisation*) Factory::getElement(pairs.second);
        t->setSynchronisation(sync);
    }
    Factory::transitionSynchReferences.clear();
    // transitionConditionReferences
    for (std::pair<int64_t, int64_t>& pairs : Factory::transitionConditionReferences) {
        Transition* t = (Transition*) Factory::getElement(pairs.first);
        TransitionCondition* transitionCondition = (TransitionCondition*) Factory::getElement(pairs.second);
        t->setTransitionCondition(transitionCondition);
    }
    Factory::transitionConditionReferences.clear();
}
} // namespace alica
