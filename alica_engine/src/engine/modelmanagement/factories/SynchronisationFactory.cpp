#include "engine/modelmanagement/factories/SynchronisationFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Synchronisation.h"

namespace alica
{
    Synchronisation* SynchronisationFactory::create(const YAML::Node& synchronisationNode, Plan* plan)
    {
        Synchronisation* synchronisation;
        // TODO analyse type of Quantifier
//        = new Quantifier();
        Factory::setAttributes(synchronisationNode, synchronisation);
        Factory::storeElement(synchronisation, alica::Strings::synchronisation);
        synchronisation->_plan = plan;

        // TODO
        std::cerr << "SynchronisationFactory: Parsing Synchronisation not completely implemented, yet!" << std::endl;

        return synchronisation;
    }
} // namespace alica
