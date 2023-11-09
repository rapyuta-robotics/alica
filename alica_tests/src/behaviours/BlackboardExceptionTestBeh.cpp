#include <alica_tests/behaviours/BlackboardExceptionTestBeh.h>
#include <engine/PlanStatus.h>
#include <memory>

namespace alica
{

BlackboardExceptionTestBeh::BlackboardExceptionTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
void BlackboardExceptionTestBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("boolValue", true);
    bb.set<PlanStatus>("planStatus", PlanStatus::Success);

    try {
        bb.get<std::string>("boolValue");
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Error: No exception in blackboard after accessing boolValue with wrong type std::string!");
        return;
    } catch (const BlackboardException& e) {
        // nothing to do
    }

    try {
        bb.get<std::string>("unknownKey");
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Error: No exception in blackboard after accessing unknown key!");
    } catch (const BlackboardException& e) {
        // nothing to do
    }

    try {
        bb.get<double>("planStatus");
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Error: No exception in blackboard after accessing custom type with wrong type!");
        return;
    } catch (const BlackboardException& e) {
        // nothing to do
    }

    try {
        bb.get<UnknownType>("planStatus");
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Error: No exception in blackboard after accessing custom type with wrong custom type!");
        return;
    } catch (const BlackboardException& e) {
        // nothing to do
    }

    try {
        bb.set("boolValue", "invalidValue");
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Error: No exception in blackboard after setting blackboard value with the wrong type!");
        return;
    } catch (const BlackboardException& e) {
        // nothing to do
    }

    // all exceptions are thrown correctly
    setSuccess();
}

std::unique_ptr<BlackboardExceptionTestBeh> BlackboardExceptionTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<BlackboardExceptionTestBeh>(context);
}

} /* namespace alica */
