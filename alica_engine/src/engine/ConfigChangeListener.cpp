#include "engine/ConfigChangeListener.h"

namespace alica
{

ConfigChangeListener::ConfigChangeListener(YAML::Node& config)
        : _config(config)
{
}

void ConfigChangeListener::subscribe(ReloadFunction reloadFunction)
{
    _configChangeListenerCBs.push_back(reloadFunction);
}

void ConfigChangeListener::reloadConfig(const YAML::Node& config)
{
    for (auto reloadFunction : _configChangeListenerCBs) {
        reloadFunction(config);
    }
}

YAML::Node& ConfigChangeListener::getConfig() const
{
    return _config;
}

} // namespace alica
