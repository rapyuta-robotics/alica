#include "engine/modelmanagement/factories/BehaviourFactory.h"

#include "engine/modelmanagement/Strings.h"
#include "engine/model/Behaviour.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/BehaviourConfigurationFactory.h"
#include "engine/model/BehaviourConfiguration.h"

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

        if (Factory::isValid(node[alica::Strings::behaviourConfigurations])) {
            const YAML::Node& behaviourConfigurations = node[alica::Strings::behaviourConfigurations];
            for (YAML::const_iterator it = behaviourConfigurations.begin(); it != behaviourConfigurations.end(); ++it) {
                BehaviourConfiguration* behaviourConf = BehaviourConfigurationFactory::create(*it, behaviour);
                behaviour->_behaviourConfigurations.push_back(behaviourConf);
                std::cout << "BehaviourFactory: " << behaviourConf->toString() << std::endl;
            }
        }

        return behaviour;
    }

    void BehaviourFactory::attachReferences() {
        ConditionFactory::attachReferences();
    }
} // namespace alica