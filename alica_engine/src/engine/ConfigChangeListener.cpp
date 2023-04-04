#include "engine/ConfigChangeListener.h"

namespace alica
{

ConfigChangeListener::ConfigChangeListener(YAML::Node& config)
        : _config(config)
        , _active(true)
{
}

void ConfigChangeListener::subscribe(ReloadFunction reloadFunction)
{
    if (!_active) {
        return;
    }
    _configChangeListenerCBs.push_back(reloadFunction);
}

void ConfigChangeListener::reloadConfig(const YAML::Node& config)
{
    if (!_active) {
        return;
    }
    for (auto reloadFunction : _configChangeListenerCBs) {
        reloadFunction(config);
    }
}

YAML::Node& ConfigChangeListener::getConfig() const
{
    return _config;
}

void ConfigChangeListener::deactivate()
{
    _active = false;
}

} // namespace alica
