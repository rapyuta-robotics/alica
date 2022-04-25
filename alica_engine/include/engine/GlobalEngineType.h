#pragma once

#include <yaml-cpp/yaml.h>
#include <functional>

using ReloadFunction = std::function<void(const YAML::Node&)>;
using SubscribeFunction = std::function<void(ReloadFunction)>;