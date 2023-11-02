#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{

class PlaceholderMapping;

class PlaceholderMappingFactory : public Factory
{
public:
    static PlaceholderMapping* create(const YAML::Node& node);
    static void attachReferences();

private:
    PlaceholderMappingFactory() = delete;
};

} // namespace alica
