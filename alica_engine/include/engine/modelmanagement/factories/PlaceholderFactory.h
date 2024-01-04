#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{

class Placeholder;

class PlaceholderFactory : public Factory
{
public:
    static Placeholder* create(const YAML::Node& node);
    static void attachReferences();

private:
    PlaceholderFactory() = delete;
};

} // namespace alica
