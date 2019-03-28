#include "engine/modelmanagement/factories/EntryPointFactory.h"
#include "engine/modelmanagement/factories/Factory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/EntryPoint.h"

namespace alica {
    std::vector<EntryPoint*> EntryPointFactory::create(const YAML::Node& entryPoints)
    {
        std::vector<EntryPoint*> constructedEntryPoints;
        for (YAML::const_iterator it = entryPoints.begin(); it != entryPoints.end(); ++it) {
            const YAML::Node& epNode = *it;

            EntryPoint* ep = new EntryPoint();
            Factory::setAttributes(epNode, ep);
            Factory::storeElement(ep, alica::Strings::entryPoint);

            ep->_plan = (Plan*) Factory::getElement(Factory::getReferencedId(epNode[alica::Strings::plan]));
            if (epNode[alica::Strings::minCardinality]) {
                ep->_cardinality.setMin(epNode[alica::Strings::minCardinality].as<int>());
            }
            if (epNode[alica::Strings::maxCardinality]) {
                ep->_cardinality.setMax(epNode[alica::Strings::maxCardinality].as<int>());
            }
            if (epNode[alica::Strings::successRequired]) {
                ep->setSuccessRequired(epNode[alica::Strings::successRequired].as<bool>());
            }
            if (epNode[alica::Strings::state]) {
                Factory::epStateReferences.push_back(std::pair<int64_t, int64_t>(ep->getId(), Factory::getReferencedId(epNode[alica::Strings::state])));
            }
            if (epNode[alica::Strings::task]) {
                Factory::epTaskReferences.push_back(std::pair<int64_t, int64_t>(ep->getId(), Factory::getReferencedId(epNode[alica::Strings::task])));
            }
            constructedEntryPoints.push_back(ep);
        }

        // SORT EntryPoints
        std::sort(constructedEntryPoints.begin(), constructedEntryPoints.end(),
                  [](const EntryPoint* ep1, const EntryPoint* ep2) { return ep1->getId() < ep2->getId(); });

        for (int i = 0; i < static_cast<int>(entryPoints.size()); ++i) {
            constructedEntryPoints[i]->_index = i;
        }

        return constructedEntryPoints;
    }

}
