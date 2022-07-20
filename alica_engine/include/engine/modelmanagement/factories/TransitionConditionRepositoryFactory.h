#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class TransitionConditionRepository;
class TransitionConditionRepositoryFactory : public Factory
{
public:
    static TransitionConditionRepository* create(const YAML::Node& node);

private:
    TransitionConditionRepositoryFactory() = delete;
};
} // namespace alica
