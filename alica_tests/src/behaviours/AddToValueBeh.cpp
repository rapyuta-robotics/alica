#include <alica_tests/behaviours/AddToValueBeh.h>
#include <memory>

namespace alica
{

AddToValueBeh::AddToValueBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
void AddToValueBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("result", bb.get<int64_t>("valueBase") + bb.get<int64_t>("valueAddition"));
    setSuccess();
}

std::unique_ptr<AddToValueBeh> AddToValueBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<AddToValueBeh>(context);
}

} /* namespace alica */
