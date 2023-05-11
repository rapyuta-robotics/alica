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
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Init\n");
}

void PlanAA::onTerminate()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Term\n");
}
} // namespace alica
