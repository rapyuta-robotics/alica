#include "engine/modelmanagement/factories/QuantifierFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Quantifier.h"

namespace alica
{
    Quantifier* QuantifierFactory::create(const YAML::Node& quantifierNode)
    {
        Quantifier* quantifier;
        // TODO analyse type of Quantifier
//        = new Quantifier();
        Factory::setAttributes(quantifierNode, quantifier);
        Factory::storeElement(quantifier, alica::Strings::quantifier);

        // TODO
        std::cerr << "QuantifierFactory: Parsing Quantifier not completely implemented, yet!" << std::endl;

        return quantifier;
    }
} // namespace alica
