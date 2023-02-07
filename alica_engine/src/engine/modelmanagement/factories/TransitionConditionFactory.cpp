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
    std::unique_ptr<BlackboardBlueprint> blackboardBlueprint;
    if (Factory::isValid(conditionNode[alica::Strings::blackboard])) {
        blackboardBlueprint = BlackboardBlueprintFactory::create(conditionNode[alica::Strings::blackboard]);
    } else {
        blackboardBlueprint = BlackboardBlueprintFactory::createEmpty();
    }
    TransitionCondition* transitionCondition = new TransitionCondition(std::move(blackboardBlueprint));
    Factory::setAttributes(conditionNode, transitionCondition);
    Factory::storeElement(transitionCondition, alica::Strings::transitionCondition);

    if (Factory::isValid(conditionNode[alica::Strings::libraryName]))
        transitionCondition->setLibraryName(Factory::getValue<std::string>(conditionNode, alica::Strings::libraryName));

    return transitionCondition;
}
} // namespace alica
