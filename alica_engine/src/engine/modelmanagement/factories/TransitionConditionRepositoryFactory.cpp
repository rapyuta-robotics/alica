#include "engine/modelmanagement/factories/TransitionConditionRepositoryFactory.h"

#include "engine/model/TransitionConditionRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

#include <iostream>
#include <string>

namespace alica
{
TransitionConditionRepository* TransitionConditionRepositoryFactory::create(const YAML::Node& node)
{
    TransitionConditionRepository* conditionRepository = new TransitionConditionRepository();
    Factory::setAttributes(node, conditionRepository);
    Factory::storeElement(conditionRepository, alica::Strings::transitionConditionRepository);

    if (Factory::isValid(node["conditions"])) {
        for (YAML::const_iterator it = node["conditions"].begin(); it != node["conditions"].end(); ++it) {
            conditionRepository->_transitionConditions.push_back(TransitionConditionFactory::create(*it, conditionRepository));
        }
    }

    return conditionRepository;
}
} // namespace alica
