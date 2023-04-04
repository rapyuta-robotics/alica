#include "engine/ConfigChangeListener.h"

namespace alica
{

ConfigChangeListener::ConfigChangeListener(YAML::Node& config)
        : _config(config)
        , _callbackIdCounter(0)
{
}

uint64_t ConfigChangeListener::subscribe(ReloadFunction reloadFunction)
{
    _callbackIdCounter++;
    _configChangeListenerCBs[_callbackIdCounter] = reloadFunction;
    return _callbackIdCounter;
}

void ConfigChangeListener::unsubscribe(uint64_t callbackId)
{
    _configChangeListenerCBs.erase(callbackId);
}

void ConfigChangeListener::reloadConfig(const YAML::Node& config)
{
    for (auto it = _configChangeListenerCBs.begin(); it != _configChangeListenerCBs.end(); it++) {
        it->second(config);
    }
}

YAML::Node& ConfigChangeListener::getConfig() const
{
    return _config;
}

} // namespace alica
