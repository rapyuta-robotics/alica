#include "engine/modelmanagement/factories/TransitionConditionRepositoryFactory.h"

#include "engine/model/TransitionConditionRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/TransitionConditionFactory.h"

#include <iostream>
#include <random>
#include <string>

namespace alica
{
TransitionConditionRepository* TransitionConditionRepositoryFactory::create(const YAML::Node& node)
{
    TransitionConditionRepository* conditionRepository = new TransitionConditionRepository();
    Factory::setAttributes(node, conditionRepository);
    Factory::storeElement(conditionRepository, alica::Strings::transitionConditionRepository);

    if (Factory::isValid(node["conditions"])) {
        for (YAML::const_iterator it = node["conditions"].begin(); it != node["conditions"].end(); ++it) {
            conditionRepository->_transitionConditions.push_back(TransitionConditionFactory::create(*it, conditionRepository));
        }
    }

    return conditionRepository;
}

TransitionConditionRepository* TransitionConditionRepositoryFactory::createAndAttach(PlanRepository& planRepository)
{
    TransitionConditionRepository* conditionRepository = new TransitionConditionRepository();
    YAML::Node node;
    node["id"] = generateID();
    Factory::setAttributes(node, conditionRepository);
    Factory::storeElement(conditionRepository, alica::Strings::transitionConditionRepository);

    for (auto it = planRepository._transitions.begin(); it != planRepository._transitions.end(); it++) {
        conditionRepository->_transitionConditions.push_back(TransitionConditionFactory::createAndAttach(conditionRepository, it->second, generateID()));
    }

    return conditionRepository;
}

int64_t TransitionConditionRepositoryFactory::generateID()
{
    std::random_device device;
    std::uniform_int_distribution<int32_t> distribution(1, std::numeric_limits<int32_t>::max());
    uint64_t id = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    id = (id << 32) | (distribution(device));
    return id;
}
} // namespace alica
