#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class ConditionRepository;
class ConditionRepositoryFactory : public Factory
{
public:
    static ConditionRepository* create(const YAML::Node& node);

private:
    ConditionRepositoryFactory() = delete;
};
} // namespace alica
