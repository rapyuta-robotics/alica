#include <alica_tests/behaviours/AccessAsAnyTestBeh.h>
#include <any>
#include <engine/PlanStatus.h>
#include <memory>

namespace alica
{

AccessAsAnyTestBeh::AccessAsAnyTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void AccessAsAnyTestBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set<PlanStatus>("anyValue", PlanStatus::Success);
    std::any anyValue = bb.get<std::any>("anyValue");
    PlanStatus status = std::any_cast<PlanStatus>(anyValue);

    if (status != PlanStatus::Success) {
        return;
    }

    setSuccess();
}

std::unique_ptr<AccessAsAnyTestBeh> AccessAsAnyTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<AccessAsAnyTestBeh>(context);
}

} /* namespace alica */
