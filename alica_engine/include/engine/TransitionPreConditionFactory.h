#pragma once

#include "engine/ITransitionPreConditionCreator.h"

#include <unordered_map>
#include <string>
#include <any>

namespace alica
{
class TransitionPreConditionFactory
{
public:
    TransitionPreConditionFactory(ITransitionPreConditionCreator& cc)
            : _creator(cc)
    {
    }
    ~TransitionPreConditionFactory() = default;

    std::function<bool(std::unordered_map<std::string, std::any>)> create(int64_t id) const
    {
        auto transitionCondition = _creator.createConditions(id);
        if (!transitionCondition) {
            // ALICA_ERROR_MSG("TransitionPreConditionFactory: Condition creation failed: " << id);
            return nullptr;
        }
        return transitionCondition;
    }

private:
    ITransitionPreConditionCreator& _creator;
};
} // namespace alica