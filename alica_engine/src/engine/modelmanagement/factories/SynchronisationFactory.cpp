#include "engine/modelmanagement/factories/SynchronisationFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Synchronisation.h"

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
        synchronisation->_syncTimeout = AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::talkTimeout));

        //Note: synchronisation->_inSync are set via resolve references with Factory::transitionSynchReferences (filled in the TransitionFactory)

        return synchronisation;
    }
} // namespace alica
