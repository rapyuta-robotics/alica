#include <alica_tests/behaviours/SetAsAnyTestBeh.h>
#include <any>
#include <engine/PlanStatus.h>
#include <memory>

namespace alica
{

SetAsAnyTestBeh::SetAsAnyTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void SetAsAnyTestBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("anyValue", std::any{true});

    if (!bb.get<bool>("anyValue")) {
        return;
    }

    setSuccess();
}

std::unique_ptr<SetAsAnyTestBeh> SetAsAnyTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<SetAsAnyTestBeh>(context);
}

} /* namespace alica */
