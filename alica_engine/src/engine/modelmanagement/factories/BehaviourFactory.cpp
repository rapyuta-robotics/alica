#include "engine/modelmanagement/factories/BehaviourFactory.h"

#include "engine/model/Behaviour.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"
#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"

namespace alica
{
Behaviour* BehaviourFactory::create(const YAML::Node& node)
{
    auto* behaviour = new Behaviour();
    Factory::setAttributes(node, behaviour);
    Factory::storeElement(behaviour, alica::Strings::behaviour);
    AbstractPlanFactory::setVariables(node, behaviour);

    behaviour->setFrequency(Factory::getValue<int>(node, alica::Strings::frequency, 1));
    behaviour->setDeferring(Factory::getValue<int>(node, alica::Strings::deferring, 0));
    behaviour->setEventDriven(Factory::getValue<bool>(node, alica::Strings::eventDriven, false));

    if (Factory::isValid(node[alica::Strings::libraryName])) {
        behaviour->setLibraryName(Factory::getValue<std::string>(node, alica::Strings::libraryName, ""));
    }
    if (Factory::isValid(node[alica::Strings::implementationName])) {
        behaviour->setImplementationName(Factory::getValue<std::string>(node, alica::Strings::implementationName, ""));
    }
    if (Factory::isValid(node[alica::Strings::preCondition])) {
        behaviour->setPreCondition(PreConditionFactory::create(node[alica::Strings::preCondition], behaviour));
    }
    if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
        behaviour->setRuntimeCondition(RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], behaviour));
    }
    if (Factory::isValid(node[alica::Strings::postCondition])) {
        behaviour->setPostCondition(PostConditionFactory::create(node[alica::Strings::postCondition], behaviour));
    }
    const auto inheritBlackboard = Factory::getValue<bool>(node, alica::Strings::inheritBlackboard, true);
    if (!inheritBlackboard) {
        if (Factory::isValid(node[alica::Strings::blackboard])) {
            behaviour->setBlackboardBlueprint(BlackboardBlueprintFactory::create(node[alica::Strings::blackboard]));
        } else {
            behaviour->setBlackboardBlueprint(BlackboardBlueprintFactory::createEmpty());
        }
    }
    return behaviour;
}

void BehaviourFactory::attachReferences()
{
    ConditionFactory::attachReferences();
}
} // namespace alica
