#pragma once

#include "engine/ITransitionConditionCreator.h"
#include "engine/DefaultTransitionConditionCreator.h"

#include <memory>

namespace alica
{
class IAlicaWorldModel;
class TransitionCondition;
class BasicTransitionCondition;

class RuntimeTransitionConditionFactory
{
public:
    RuntimeTransitionConditionFactory(std::unique_ptr<ITransitionConditionCreator>&& tcc, IAlicaWorldModel* wm);
    ~RuntimeTransitionConditionFactory() = default;

    std::unique_ptr<BasicTransitionCondition> create(const TransitionCondition* transitionConditionModel) const;

private:
    std::unique_ptr<ITransitionConditionCreator> _creator;
    DefaultTransitionConditionCreator _defaultTransitionConditionCreator;
    IAlicaWorldModel* _wm;
};
} // namespace alica