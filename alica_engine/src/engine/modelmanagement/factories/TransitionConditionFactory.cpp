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
        blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(conditionNode[alica::Strings::blackboard]));
    } else {
        blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
    }

    TransitionConditionContext ctx{std::move(blackboardBlueprint)};
    TransitionCondition* transitionCondition = new TransitionCondition(ctx);
    Factory::setAttributes(conditionNode, transitionCondition);
    Factory::storeElement(transitionCondition, alica::Strings::transitionCondition);

    return transitionCondition;

}
} // namespace alica
