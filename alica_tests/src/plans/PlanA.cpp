#include <alica_tests/plans/PlanA.h>

namespace alica
{
PlanA::PlanA(PlanContext& context)
        : AlicaTestsPlan(context)
{
    _inRunContext = false;
}
void PlanA::onInit()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("execOrder", (gb.hasValue("execOrder") ? gb.get<std::string>("execOrder") : "") + getName() + "::Init\n");
    gb.set("counter", (gb.hasValue("counter") ? gb.get<int64_t>("counter") + 1 : 1));
    _inRunContext = true;
}

void PlanA::run()
{
    if (!_inRunContext) {
        LockedBlackboardRW(*getGlobalBlackboard()).set(getName() + "RunOutOfOrder", true);
    } else {
        LockedBlackboardRW(*getGlobalBlackboard()).set(getName() + "RunCalled", true);
    }
}

void PlanA::onTerminate()
{
    _inRunContext = false;
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Term\n");
}

} // namespace alica
