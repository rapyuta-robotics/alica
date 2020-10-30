#pragma once

#include <yaml-cpp/yaml.h>

namespace alica
{
class ConfigInitializer
{
public:
    ConfigInitializer(std::string configPath = "/etc", std::string configName = "Alica");
    YAML::Node* loadConfig();

private:
    std::string _configPath;
    std::string _configName;
};
} /* namespace alica */
