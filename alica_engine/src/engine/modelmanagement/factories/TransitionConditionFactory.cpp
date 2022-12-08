#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

#include "engine/blackboard/KeyMapping.h"
#include "engine/model/Transition.h"
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
        blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(conditionNode[alica::Strings::blackboard]));
    } else {
        blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
    }
    TransitionCondition* transitionCondition = new TransitionCondition(std::move(blackboardBlueprint));
    Factory::setAttributes(conditionNode, transitionCondition);
    Factory::storeElement(transitionCondition, alica::Strings::transitionCondition);

    if (Factory::isValid(conditionNode[alica::Strings::libraryName]))
        transitionCondition->setLibraryName(Factory::getValue<std::string>(conditionNode, alica::Strings::libraryName));

    return transitionCondition;
}
TransitionCondition* TransitionConditionFactory::createAndAttach(TransitionConditionRepository* conditionRepository, Transition* transition, int64_t id)
{
    TransitionCondition* transitionCondition = new TransitionCondition(std::move(BlackboardBlueprintFactory::createEmpty()));
    YAML::Node node;
    node["id"] = id;
    Factory::setAttributes(node, transitionCondition);
    Factory::storeElement(transitionCondition, alica::Strings::transitionCondition);
    transition->setTransitionCondition(transitionCondition);
    transition->_keyMapping = std::make_unique<KeyMapping>(); // old transitions dont have keyMapping, set empty keymapping
    return transitionCondition;
}
} // namespace alica
