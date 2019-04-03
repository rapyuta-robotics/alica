#include "engine/modelmanagement/factories/QuantifierFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Quantifier.h"
#include "engine/model/ForallAgents.h"

namespace alica
{
    Quantifier* QuantifierFactory::create(const YAML::Node& quantifierNode)
    {
        Quantifier* quantifier;
        std::string quantifierType;
        if (Factory::isValid(quantifierNode[alica::Strings::quantifierType])) {
            quantifierType = Factory::getValue<std::string>(quantifierNode, alica::Strings::quantifierType);
        }
        if (quantifierType.compare(alica::Strings::forAllAgents) == 0) {
            quantifier = new ForallAgents();
        } else {
            AlicaEngine::abort("QuantifierFactory: Unknown quantifier type encountered!");
        }
        Factory::setAttributes(quantifierNode, quantifier);
        Factory::storeElement(quantifier, alica::Strings::quantifier);

        if (Factory::isValid(quantifierNode[alica::Strings::scope])) {
            Factory::quantifierScopeReferences.push_back(std::pair<int64_t, int64_t>(quantifier->getId(), Factory::getReferencedId(quantifierNode[alica::Strings::scope])));
        }

        if (Factory::isValid(quantifierNode[alica::Strings::sorts])) {
            const YAML::Node &sorts = quantifierNode[alica::Strings::sorts];
            for (YAML::const_iterator it = sorts.begin(); it != sorts.end(); ++it) {
                quantifier->_domainIdentifiers.push_back((*it).as<std::string>());
            }
        }

        return quantifier;
    }

    void QuantifierFactory::attachReferences() {
        // quantifierScopeReferences
        for (std::pair<int64_t, int64_t> pairs : Factory::quantifierScopeReferences) {
            AlicaElement* ael = (AlicaElement*) Factory::getElement(pairs.second);
            Quantifier* q = (Quantifier*) Factory::getElement(pairs.first);
            q->setScope(ael);
        }
        Factory::quantifierScopeReferences.clear();
    }
} // namespace alica
