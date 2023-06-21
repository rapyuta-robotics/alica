#include <alica_tests/plans/PlanAA.h>

namespace alica
{
PlanAA::PlanAA(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void PlanAA::onInit()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Init");
}

void PlanAA::onTerminate()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Term");
}
} // namespace alica
