#include "engine/TransitionConditionCallbackFactory.h"

#include "engine/model/TransitionCondition.h"
#include "engine/IAlicaWorldModel.h"

#include <alica_common_config/debug_output.h>
#include <iostream>
#include <string>

namespace alica
{
    TransitionConditionCallbackFactory::TransitionConditionCallbackFactory(std::unique_ptr<ITransitionConditionCreator>&& tcc, IAlicaWorldModel* wm)
            : _creator(std::move(tcc))
            , _wm(wm)
    {
    }

    TransitionConditionCallback TransitionConditionCallbackFactory::create(const TransitionCondition* transitionConditionModel) const
    {
        if (_defaultTransitionConditionCreator.isDefaultTransitionCondition(transitionConditionModel->getName())) {
            return _defaultTransitionConditionCreator.createConditions(transitionConditionModel->getName());
        } else {
            return _creator->createConditions(transitionConditionModel->getId());
        }
        ALICA_ERROR_MSG("TransitionConditionCallbackFactory: Condition creation failed: " << transitionConditionModel->getId());
        return TransitionConditionCallback();
    }
} // namespace alica