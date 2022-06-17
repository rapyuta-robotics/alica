#include "engine/model/TransitionCondition.h"

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
TransitionCondition::TransitionCondition(std::unique_ptr<BlackboardBlueprint> blackboardBlueprint)
        : _blackboard(std::make_unique<Blackboard>(blackboardBlueprint.get())) {}

bool TransitionCondition::evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping)
{
    if (rp->isBehaviour()) {
        return false;
    }
    assert(_evalCallback);
    keyMapping->setInput(rp->getBasicPlan()->getBlackboard().get(), _blackboard.get());
    return _evalCallback(_blackboard.get(), rp, wm);
}
} /* namespace alica */
