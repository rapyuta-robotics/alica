#pragma once

#include <functional>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "engine/Types.h"

namespace alica
{

class ConfigChangeListener
{
public:
    using ReloadFunction = std::function<void(const YAML::Node&)>;
    using ConfigChangeSubscriber = std::function<void(ReloadFunction)>;

    ConfigChangeListener(YAML::Node& config);
    void subscribe(ReloadFunction reloadFunction);
    void reloadConfig(const YAML::Node& config);
    YAML::Node& getConfig() const;

private:
    std::vector<ReloadFunction> _configChangeListenerCBs;
    YAML::Node& _config;
};
} // namespace alica
