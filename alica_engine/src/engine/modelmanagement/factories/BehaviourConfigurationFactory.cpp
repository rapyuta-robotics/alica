#include "engine/modelmanagement/factories/BehaviourConfigurationFactory.h"

#include "engine/model/BehaviourConfiguration.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"

namespace alica
{

BehaviourConfiguration* BehaviourConfigurationFactory::create(const YAML::Node& node)
{
    BehaviourConfiguration* behaviourConf = new BehaviourConfiguration();
    Factory::setAttributes(node, behaviourConf);
    Factory::storeElement(behaviourConf, alica::Strings::behaviourConfiguration);

    if (Factory::isValid(node[alica::Strings::keyValuePairs])) {
        const YAML::Node &keyValuePairs = node[alica::Strings::keyValuePairs];
        for (YAML::const_iterator it = keyValuePairs.begin(); it != keyValuePairs.end(); ++it) {
            behaviourConf->_parameters.insert(std::pair<std::string, std::string>(it->first.as<std::string>(), it->second.as<std::string>()));
        }
    }

    return behaviourConf;
}
} // namespace alica