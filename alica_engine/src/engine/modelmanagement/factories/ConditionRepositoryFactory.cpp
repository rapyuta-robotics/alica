#include "engine/modelmanagement/factories/ConditionRepositoryFactory.h"

#include "engine/model/Condition.h"
#include "engine/model/ConditionRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

namespace alica
{
ConditionRepository* ConditionRepositoryFactory::create(const YAML::Node& node)
{
    ConditionRepository* conditionRepository = new ConditionRepository();
    Factory::setAttributes(node, conditionRepository);
    Factory::storeElement(conditionRepository, alica::Strings::conditionrepository);

    if (Factory::isValid(node)) {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            conditionRepository->_conditions.push_back(TransitionConditionFactory::create(*it, conditionRepository));
        }
    }

    return conditionRepository;
}
} // namespace alica
