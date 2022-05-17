#include "engine/modelmanagement/factories/TransitionConditionFactory.h"
#include "engine/model/TransitionCondition.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
TransitionCondition* TransitionConditionFactory::create(const YAML::Node& conditionNode, ConditionRepository* conditionRepository)
{
    TransitionCondition* transitionCondition = new TransitionCondition();
    Factory::setAttributes(conditionNode, transitionCondition);
    Factory::storeElement(condition, alica::Strings::transitionCondition);
    transitionCondition->_conditionRepository = conditionRepository;

    if (Factory::isValid(node[alica::Strings::blackboard])) {
        transitionCondition->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(node[alica::Strings::blackboard]));
    } else {
        transitionCondition->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
    }
    return transitionCondition;

}
} // namespace alica
