#include "engine/BasicTransitionCondition.h"

#include "engine/AlicaEngine.h"
#include "engine/RunningPlan.h"

namespace alica
{

BasicTransitionCondition::BasicTransitionCondition(TransitionConditionContext& context)
        : RunnableObject(context.worldModel, context.name)
        , _transitionCondition(context.transitionConditionModel)
        , _evalCallback(context.evalCallback) {}

BasicTransitionCondition::~BasicTransitionCondition() {}

bool BasicTransitionCondition::evaluate(RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return false;
    }
    setInput(rp->getBasicPlan()->getBlackboard().get(), rp->getKeyMapping(getParentWrapperId(rp)));
    return _evalCallback(rp, getBlackboard().get());
}
} /* namespace alica */
