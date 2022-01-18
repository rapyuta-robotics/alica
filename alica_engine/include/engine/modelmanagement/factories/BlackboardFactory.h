#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class Blackboard;
class BlackboardFactory : public Factory
{
public:
    static std::unique_ptr<Blackboard> create(const YAML::Node& node);
    static std::unique_ptr<Blackboard> createEmpty();

private:
    BlackboardFactory() = delete;
};
} // namespace alica
