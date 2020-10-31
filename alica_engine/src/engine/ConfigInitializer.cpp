#include "engine/ConfigInitializer.h"

namespace alica
{
    YAML::Node ConfigInitializer::loadConfig(std::string configPath, std::string configName)
    {
        YAML::Node node;
        try {
            node = YAML::LoadFile(configPath + "/" + configName + ".yaml");
        } catch (YAML::BadFile& badFile) {
            AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
        }
        return node;
    }

} /* namespace alica */
