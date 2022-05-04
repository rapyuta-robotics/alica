#include "engine/ConfigChangeListener.h"

namespace alica
{

void ConfigChangeListener::subscribe(ReloadFunction reloadFunction)
{
    _configChangeListenerCBs.push_back(reloadFunction);
};

void ConfigChangeListener::reloadConfig(const YAML::Node& config)
{
    for (auto reloadFunction : _configChangeListenerCBs) {
        reloadFunction(config);
    }
}

} // namespace alica
