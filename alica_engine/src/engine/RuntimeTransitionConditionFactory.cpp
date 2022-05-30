#include "engine/RuntimeTransitionConditionFactory.h"

#include "engine/BasicTransitionCondition.h"
#include "engine/model/TransitionCondition.h"
#include "engine/Types.h"
#include "engine/DefaultTransitionConditionCreator.h"

#include <alica_common_config/debug_output.h>
#include <iostream>
#include <string>

namespace alica
{
    RuntimeTransitionConditionFactory::RuntimeTransitionConditionFactory(std::unique_ptr<ITransitionConditionCreator>&& tcc, IAlicaWorldModel* wm)
            : _creator(std::move(tcc))
            , _wm(wm)
    {
    }

    std::unique_ptr<BasicTransitionCondition> RuntimeTransitionConditionFactory::create(const TransitionCondition* transitionConditionModel) const
    {
        TransitionConditionCallback evalCallback;
        if (_defaultTransitionConditionCreator.isDefaultTransitionCondition(transitionConditionModel->getName())) {
            evalCallback = _defaultTransitionConditionCreator.createConditions(transitionConditionModel->getName());
        } else {
            evalCallback = _creator->createConditions(transitionConditionModel->getId());
        }
        if (!evalCallback) {
            ALICA_ERROR_MSG("RuntimeTransitionConditionFactory: Condition creation failed: " << transitionConditionModel->getId());
            return nullptr;
        }
        TransitionConditionContext ctx{transitionConditionModel, evalCallback};
        std::unique_ptr<BasicTransitionCondition> basicTransitionCondition = std::make_unique<BasicTransitionCondition>(ctx);
        return basicTransitionCondition;
    }
} // namespace alica