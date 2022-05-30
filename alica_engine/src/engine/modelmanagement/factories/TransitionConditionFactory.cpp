#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

#include "engine/model/TransitionCondition.h"
#include "engine/model/TransitionConditionRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

#include <string>

namespace alica
{
TransitionCondition* TransitionConditionFactory::create(const YAML::Node& conditionNode, TransitionConditionRepository* conditionRepository)
{
    TransitionCondition* transitionCondition = new TransitionCondition();
    Factory::setAttributes(conditionNode, transitionCondition);
    Factory::storeElement(transitionCondition, alica::Strings::transitionCondition);

    if (Factory::isValid(conditionNode[alica::Strings::blackboard])) {
        transitionCondition->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(conditionNode[alica::Strings::blackboard]));
    } else {
        transitionCondition->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
    }
    return transitionCondition;

}
} // namespace alica
