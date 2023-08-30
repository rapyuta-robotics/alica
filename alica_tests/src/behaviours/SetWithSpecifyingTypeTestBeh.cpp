#include <alica_tests/behaviours/SetWithSpecifyingTypeTestBeh.h>
#include <any>
#include <engine/PlanStatus.h>
#include <memory>

namespace alica
{

SetWithSpecifyingTypeTestBeh::SetWithSpecifyingTypeTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void SetWithSpecifyingTypeTestBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set<PlanStatus>("anyValue", PlanStatus::Success);

    if (bb.get<PlanStatus>("anyValue") != PlanStatus::Success) {
        return;
    }

    setSuccess();
}

std::unique_ptr<SetWithSpecifyingTypeTestBeh> SetWithSpecifyingTypeTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<SetWithSpecifyingTypeTestBeh>(context);
}

} /* namespace alica */
