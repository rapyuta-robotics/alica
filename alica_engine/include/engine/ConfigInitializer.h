#pragma once

#include <yaml-cpp/yaml.h>
#include "engine/AlicaEngine.h"

namespace alica
{
class ConfigInitializer
{
public:
    ConfigInitializer() = default;
    YAML::Node loadConfig(std::string configPath, std::string configName);
};
} /* namespace alica */
