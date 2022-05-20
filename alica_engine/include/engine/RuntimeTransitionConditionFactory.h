#pragma once

#include "engine/ITransitionPreConditionCreator.h"

#include <memory>

namespace alica
{
class IAlicaWorldModel;
class TransitionCondition;
class BasicTransitionCondition;

class RuntimeTransitionConditionFactory
{
public:
    RuntimeTransitionConditionFactory(std::unique_ptr<ITransitionPreConditionCreator>&& cc, IAlicaWorldModel* wm);
    ~RuntimeTransitionConditionFactory() = default;

    std::unique_ptr<BasicTransitionCondition> create(const TransitionCondition* transitionConditionModel) const;

private:
    std::unique_ptr<ITransitionPreConditionCreator> _creator;
    IAlicaWorldModel* _wm;
};
} // namespace alica