#include "engine/BasicTransitionCondition.h"

#include "engine/AlicaEngine.h"
#include "engine/RunningPlan.h"
#include "engine/BasicPlan.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/model/PlanType.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/TransitionCondition.h"

namespace alica
{

BasicTransitionCondition::BasicTransitionCondition(TransitionConditionContext& context)
        : _transitionCondition(context.transitionConditionModel)
        , _evalCallback(context.evalCallback)
        , _blackboard(std::make_unique<Blackboard>(context.transitionConditionModel->getBlackboardBlueprint())) {}

BasicTransitionCondition::~BasicTransitionCondition() {}

bool BasicTransitionCondition::evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping)
{
    if (rp->isBehaviour()) {
        return false;
    }
    keyMapping->setInput(rp->getBasicPlan()->getBlackboard().get(), _blackboard.get(), keyMapping);
    return _evalCallback(_blackboard.get(), rp, wm);
}
} /* namespace alica */
