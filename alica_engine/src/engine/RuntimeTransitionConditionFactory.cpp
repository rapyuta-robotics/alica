#include "engine/RuntimeTransitionConditionFactory.h"

#include "engine/ITransitionPreConditionCreator.h"
#include "engine/BasicTransitionCondition.h"
#include "engine/model/TransitionCondition.h"


namespace alica
{
    RuntimeTransitionConditionFactory::RuntimeTransitionConditionFactory(std::unique_ptr<ITransitionPreConditionCreator>&& cc, IAlicaWorldModel* wm)
            : _creator(std::move(cc))
            , _wm(wm)
    {
    }

    std::unique_ptr<BasicTransitionCondition> RuntimeTransitionConditionFactory::create(const TransitionCondition* transitionConditionModel) const
    {
        auto evalCallback = _creator->createConditions(transitionConditionModel->getId());
        if (!evalCallback) {
            ALICA_ERROR_MSG("RuntimeTransitionConditionFactory: Condition creation failed: " << transitionConditionModel->getId());
            return nullptr;
        }
        TransitionConditionContext ctx{_wm, transitionConditionModel->getName(), transitionConditionModel, evalCallback};
        std::unique_ptr<BasicTransitionCondition> basicTransitionCondition = std::make_unique<BasicTransitionCondition>(ctx);
        return basicTransitionCondition;
    }
} // namespace alica