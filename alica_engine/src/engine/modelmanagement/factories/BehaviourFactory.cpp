#include "engine/modelmanagement/factories/BehaviourFactory.h"

#include "engine/model/Behaviour.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
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
    behaviour->_requiresParameters = Factory::getValue<bool>(node, alica::Strings::requiresParameters, false);

    if (Factory::isValid(node[alica::Strings::preCondition])) {
        behaviour->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
        behaviour->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], behaviour);
    }
    if (Factory::isValid(node[alica::Strings::postCondition])) {
        behaviour->_postCondition = PostConditionFactory::create(node[alica::Strings::postCondition], behaviour);
    }

    return behaviour;
}

void BehaviourFactory::attachReferences()
{
    ConditionFactory::attachReferences();
}
} // namespace alica