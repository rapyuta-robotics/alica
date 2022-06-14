#pragma once

#include "engine/ITransitionConditionCreator.h"
#include "engine/default/DefaultTransitionConditionCreator.h"
#include "engine/Types.h"

#include <memory>

namespace alica
{
class IAlicaWorldModel;
class TransitionCondition;
class BasicTransitionCondition;

class TransitionConditionCallbackFactory
{
public:
    TransitionConditionCallbackFactory(std::unique_ptr<ITransitionConditionCreator>&& tcc, IAlicaWorldModel* wm);
    ~TransitionConditionCallbackFactory() = default;

    TransitionConditionCallback create(const TransitionCondition* transitionConditionModel) const;

private:
    std::unique_ptr<ITransitionConditionCreator> _creator;
    DefaultTransitionConditionCreator _defaultTransitionConditionCreator;
    IAlicaWorldModel* _wm;
};
} // namespace alica