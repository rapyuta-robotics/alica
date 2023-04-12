#pragma once

#include <functional>
#include <unordered_map>
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
    uint64_t subscribe(ReloadFunction reloadFunction);
    void unsubscribe(uint64_t callbackId);
    void reloadConfig(const YAML::Node& config);
    YAML::Node& getConfig() const;

private:
    std::unordered_map<uint64_t, ReloadFunction> _configChangeListenerCBs;
    YAML::Node& _config;
    uint64_t _callbackIdCounter;
};
} // namespace alica
