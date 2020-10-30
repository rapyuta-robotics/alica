#include "ConfigInitializer.h"

namespace alica
{
    ConfigInitializer::ConfigInitializer(std::string configPath = "/etc", std::string configName = "Alica")
    {
        _configPath = configPath;
        _configName = configName;
    }

    YAML::Node* ConfigInitializer::loadConfig()
    {
        YAML::Node* node;
        try {
            node = new YAML::LoadFile(_configPath + "/" + _configName + ".yaml");
        } catch (YAML::BadFile& badFile) {
            AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
        }
        return node;
    }

} /* namespace alica */
