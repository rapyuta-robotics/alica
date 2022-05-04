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
    void subscribe(ReloadFunction reloadFunction);
    void reloadConfig(const YAML::Node& config);

private:
    std::vector<ReloadFunction> _configChangeListenerCBs;
};
}