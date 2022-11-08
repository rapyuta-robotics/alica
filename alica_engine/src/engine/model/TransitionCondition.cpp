#include "engine/model/TransitionCondition.h"

#include "engine/BasicPlan.h"
#include "engine/RunningPlan.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/BlackboardUtil.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/PlanType.h"
#include "engine/model/TransitionCondition.h"

namespace alica
{
TransitionCondition::TransitionCondition(std::unique_ptr<BlackboardBlueprint> blackboardBlueprint)
        : _blackboard(std::make_unique<Blackboard>(blackboardBlueprint.get()))
        , _libraryName("")
{
}

bool TransitionCondition::evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping)
{
    if (rp->isBehaviour()) {
        return false;
    }
    assert(_evalCallback);
    BlackboardUtil::setInput(rp->getBasicPlan()->getBlackboard().get(), _blackboard.get(), keyMapping);
    return _evalCallback(_blackboard.get(), rp, wm);
}

std::string TransitionCondition::getLibraryName() const
{
    return _libraryName;
}

void TransitionCondition::setLibraryName(const std::string& libraryname)
{
    _libraryName = libraryname;
}
} /* namespace alica */
