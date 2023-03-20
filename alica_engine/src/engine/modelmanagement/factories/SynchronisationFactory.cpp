#include "engine/modelmanagement/factories/SynchronisationFactory.h"
#include "engine/model/Synchronisation.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Synchronisation* SynchronisationFactory::create(const YAML::Node& synchronisationNode, Plan* plan)
{
    auto* synchronisation = new Synchronisation(plan, AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::talkTimeout)),
            AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::syncTimeout)),
            Factory::getValue<bool>(synchronisationNode, alica::Strings::failOnSyncTimeout));
    Factory::setAttributes(synchronisationNode, synchronisation);
    Factory::storeElement(synchronisation, alica::Strings::synchronisation);

    if (Factory::isValid(synchronisationNode[alica::Strings::inSync])) {
        const YAML::Node& transitions = synchronisationNode[alica::Strings::inSync];
        for (YAML::const_iterator it = transitions.begin(); it != transitions.end(); ++it) {
            Factory::synchTransitionReferences.push_back(std::pair<int64_t, int64_t>(synchronisation->getId(), Factory::getReferencedId(*it)));
        }
    }

    return synchronisation;
}

void SynchronisationFactory::attachReferences()
{
    // synchTransitionReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::synchTransitionReferences) {
        Synchronisation* s = (Synchronisation*) Factory::getElement(pairs.first);
        Transition* t = (Transition*) Factory::getElement(pairs.second);
        s->addInSync(t);
    }
    Factory::synchTransitionReferences.clear();
}
} // namespace alica
