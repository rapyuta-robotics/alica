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
    if (gb.hasValue("execOrder")) {
        std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
        execOrder.emplace_back(getName() + "::Init");
    } else {
        std::vector<std::string> execOrder;
        execOrder.emplace_back(getName() + "::Init");
        gb.set("execOrder", execOrder);
    }
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
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Term");
}

} // namespace alica
