#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class Behaviour;
class BehaviourFactory : public Factory
{
public:
    //[[deprecated("It will be removed in the last PR")]]
    static Behaviour* create(AlicaEngine* ae, const YAML::Node& node); // TOBE removed
    static Behaviour* create(const YAML::Node& config,  const YAML::Node& node);

    static void attachReferences();

private:
    BehaviourFactory() = delete;
};
} // namespace alica
