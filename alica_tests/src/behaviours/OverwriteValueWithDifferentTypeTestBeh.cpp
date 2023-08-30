#include <alica_tests/behaviours/OverwriteValueWithDifferentTypeTestBeh.h>
#include <any>
#include <engine/PlanStatus.h>
#include <memory>

namespace alica
{

OverwriteValueWithDifferentTypeTestBeh::OverwriteValueWithDifferentTypeTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
void OverwriteValueWithDifferentTypeTestBeh::initialiseParameters()
{
    // LockedBlackboardRW bb(*getBlackboard());
    // bb.set("anyValue", PlanStatus::Success);

    // if (bb.get<PlanStatus>("anyValue") != PlanStatus::Success) {
    //     return;
    // }

    // bb.set("anyValue", UnknownType(15));
    // if (bb.get<UnknownType>("anyValue").value != 15) {
    //     return;
    // }

    setSuccess();
}

std::unique_ptr<OverwriteValueWithDifferentTypeTestBeh> OverwriteValueWithDifferentTypeTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<OverwriteValueWithDifferentTypeTestBeh>(context);
}

} /* namespace alica */
