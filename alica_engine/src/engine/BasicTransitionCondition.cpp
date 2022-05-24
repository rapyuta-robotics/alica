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

bool BasicTransitionCondition::evaluate(RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return false;
    }
    updateInputs(rp->getBasicPlan()->getBlackboard().get(), rp->getKeyMapping(getParentWrapperId(rp)));
    return _evalCallback(rp, _blackboard.get());
}

void BasicTransitionCondition::updateInputs(const Blackboard* parentBb, const KeyMapping* keyMapping)
{
    const auto lockedParentBb = LockedBlackboardRO(*parentBb);
    auto childBb = _blackboard->impl();
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        try {
            childBb.set(childKey, lockedParentBb.get(parentKey));
        } catch (std::exception& e) {
            ALICA_WARNING_MSG("Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what());
        }
    }
}

int64_t BasicTransitionCondition::getParentWrapperId(RunningPlan* rp) const
{
    const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
    std::string name = rp->getActivePlanAsPlan()->getName();

    auto it = std::find_if(wrappers.begin(), wrappers.end(), [name](const auto& wrapper_ptr) {
        if (const auto planType = dynamic_cast<const PlanType*>(wrapper_ptr->getAbstractPlan()); planType) {
            const auto& plans = planType->getPlans();
            return std::find_if(plans.begin(), plans.end(), [name](const auto& plan) { return plan->getName() == name; }) != plans.end();
        } else {
            return wrapper_ptr->getAbstractPlan()->getName() == name;
        }
    });
    assert(it != wrappers.end());
    int64_t wrapperId = (*it)->getId();
    return wrapperId;
}
} /* namespace alica */
