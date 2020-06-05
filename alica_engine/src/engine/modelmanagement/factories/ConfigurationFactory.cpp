#include "engine/modelmanagement/factories/ConfigurationFactory.h"

#include "engine/model/Configuration.h"
#include "engine/model/Parameter.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Configuration* ConfigurationFactory::create(const YAML::Node& wrapperNode)
{
    Configuration* configuration = new Configuration();
    Factory::setAttributes(wrapperNode, configuration);
    Factory::storeElement(configuration, alica::Strings::configuration);

    if (Factory::isValid(wrapperNode[alica::Strings::parameters])) {
        const YAML::Node& keyValuePairs = wrapperNode[alica::Strings::parameters];
        for (YAML::const_iterator it = keyValuePairs.begin(); it != keyValuePairs.end(); ++it) {
            Parameter* parameter = new Parameter();
            parameter->setKey(it->first.as<std::string>());
            parameter->setValue(it->second.as<std::string>());
            configuration->_parameters.emplace(parameter->getKey(), parameter);
        }
    }

    return configuration;
}
} // namespace alica