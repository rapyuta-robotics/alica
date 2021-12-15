#include "engine/modelmanagement/factories/SynchronisationFactory.h"
#include "engine/model/Synchronisation.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Synchronisation* SynchronisationFactory::create(const YAML::Node& synchronisationNode, Plan* plan)
{
    Synchronisation* synchronisation = new Synchronisation();
    Factory::setAttributes(synchronisationNode, synchronisation);
    Factory::storeElement(synchronisation, alica::Strings::synchronisation);
    synchronisation->_plan = plan;

    synchronisation->_failOnSyncTimeout = Factory::getValue<bool>(synchronisationNode, alica::Strings::failOnSyncTimeout);
    synchronisation->_syncTimeout = AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::syncTimeout));
    synchronisation->_talkTimeout = AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::talkTimeout));

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
        s->_inSync.push_back(t);
    }
    Factory::synchTransitionReferences.clear();
}
} // namespace alica
