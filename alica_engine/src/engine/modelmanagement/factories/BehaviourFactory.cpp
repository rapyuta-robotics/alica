#include "engine/modelmanagement/factories/BehaviourFactory.h"

#include "engine/modelmanagement/Strings.h"
#include "engine/model/Behaviour.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/BehaviourConfigurationFactory.h"

namespace alica
{
    Behaviour* BehaviourFactory::create(const YAML::Node& node)
    {
        Behaviour* behaviour = new Behaviour();
        Factory::setAttributes(node, behaviour);
        Factory::storeElement(behaviour, alica::Strings::behaviour);
        AbstractPlanFactory::setVariables(node, behaviour);

        if (Factory::isValid(node[alica::Strings::frequency])) {
            behaviour->_frequency = Factory::getValue(node, alica::Strings::frequency, 30);
        }
        if (Factory::isValid(node[alica::Strings::deferring])) {
            behaviour->_deferring = Factory::getValue(node, alica::Strings::deferring, 0);
        }
        if (Factory::isValid(node[alica::Strings::preCondition])) {
            behaviour->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], behaviour);
        }
        if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
            behaviour->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], behaviour);
        }
        if (Factory::isValid(node[alica::Strings::postCondition])) {
            behaviour->_postCondition = PostConditionFactory::create(node[alica::Strings::postCondition], behaviour);
        }

        if (Factory::isValid(node[alica::Strings::parameters])) {
            const YAML::Node &keyValuePairs = node[alica::Strings::parameters];
            for (YAML::const_iterator it = keyValuePairs.begin(); it != keyValuePairs.end(); ++it) {
                behaviour->_parameters.insert(std::pair<std::string, std::string>(it->first.as<std::string>(), it->second.as<std::string>()));
            }
        }

        return behaviour;
    }

    void BehaviourFactory::attachReferences() {
        ConditionFactory::attachReferences();
    }
} // namespace alica