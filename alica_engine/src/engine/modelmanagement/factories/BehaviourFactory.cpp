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
Behaviour* BehaviourFactory::create(AlicaEngine* ae, const YAML::Node& node)
{
    Behaviour* behaviour = new Behaviour(ae);
    Factory::setAttributes(node, behaviour);
    Factory::storeElement(behaviour, alica::Strings::behaviour);
    AbstractPlanFactory::setVariables(node, behaviour);

    behaviour->_frequency = Factory::getValue<int>(node, alica::Strings::frequency, 1);
    behaviour->_deferring = Factory::getValue<int>(node, alica::Strings::deferring, 0);
    behaviour->_eventDriven = Factory::getValue<bool>(node, alica::Strings::eventDriven, false);
    behaviour->_forceLoad = Factory::getValue<bool>(node, alica::Strings::forceLoad, false);//luca
    behaviour->_companyName = Factory::getValue<std::string>(node, alica::Strings::companyName, "");//luca
    behaviour->_libraryName = Factory::getValue<std::string>(node, alica::Strings::libraryName, "");//luca
    
    if (Factory::isValid(node[alica::Strings::preCondition])) {
        behaviour->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
        behaviour->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::postCondition])) {
        behaviour->_postCondition = PostConditionFactory::create(node[alica::Strings::postCondition], behaviour);
    }
    auto inheritBlackboard = Factory::getValue<bool>(node, alica::Strings::inheritBlackboard, true);
    if (!inheritBlackboard) {
        if (Factory::isValid(node[alica::Strings::blackboard])) {
            behaviour->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(node[alica::Strings::blackboard]));
        } else {
            behaviour->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
        }
    } else {
        behaviour->_blackboardBlueprint = nullptr;
    }
    return behaviour;
}

Behaviour* BehaviourFactory::create(const YAML::Node& node)
{
    Behaviour* behaviour = new Behaviour();
    Factory::setAttributes(node, behaviour);
    Factory::storeElement(behaviour, alica::Strings::behaviour);
    AbstractPlanFactory::setVariables(node, behaviour);

    behaviour->_frequency = Factory::getValue<int>(node, alica::Strings::frequency, 1);
    behaviour->_deferring = Factory::getValue<int>(node, alica::Strings::deferring, 0);
    behaviour->_eventDriven = Factory::getValue<bool>(node, alica::Strings::eventDriven, false);
    behaviour->_forceLoad = Factory::getValue<bool>(node, alica::Strings::forceLoad, false);
    behaviour->_companyName = Factory::getValue<bool>(node, alica::Strings::companyName, false);    

    if (Factory::isValid(node[alica::Strings::preCondition])) {
        behaviour->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
        behaviour->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::postCondition])) {
        behaviour->_postCondition = PostConditionFactory::create(node[alica::Strings::postCondition], behaviour);
    }
    auto inheritBlackboard = Factory::getValue<bool>(node, alica::Strings::inheritBlackboard, true);
    if (!inheritBlackboard) {
        if (Factory::isValid(node[alica::Strings::blackboard])) {
            behaviour->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::create(node[alica::Strings::blackboard]));
        } else {
            behaviour->_blackboardBlueprint = std::move(BlackboardBlueprintFactory::createEmpty());
        }
    } else {
        behaviour->_blackboardBlueprint = nullptr;
    }
    return behaviour;
}

void BehaviourFactory::attachReferences()
{
    ConditionFactory::attachReferences();
}
} // namespace alica
