#pragma once

#include "Factory.h"
#include "engine/PlanRepository.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class TransitionConditionRepository;
class TransitionConditionRepositoryFactory : public Factory
{
public:
    static TransitionConditionRepository* create(const YAML::Node& node);

    // Used for legacy transition condition support
    static TransitionConditionRepository* createAndAttach(PlanRepository& planRepository);

private:
    TransitionConditionRepositoryFactory() = delete;
    // helper function used to generate unique IDs for legacy condition repository and legacy transition conditions
    static uint64_t generateID();
};
} // namespace alica
