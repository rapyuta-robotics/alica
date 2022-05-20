#include "engine/modelmanagement/factories/TransitionConditionRepositoryFactory.h"

#include "engine/model/TransitionConditionRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

namespace alica
{
TransitionConditionRepository* TransitionConditionRepositoryFactory::create(const YAML::Node& node)
{
    TransitionConditionRepository* conditionRepository = new TransitionConditionRepository();
    Factory::setAttributes(node, conditionRepository);
    Factory::storeElement(conditionRepository, alica::Strings::transitionConditionRepository);

    if (Factory::isValid(node)) {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            conditionRepository->_transitionConditions.push_back(TransitionConditionFactory::create(*it, conditionRepository));
        }
    }

    return conditionRepository;
}
} // namespace alica
