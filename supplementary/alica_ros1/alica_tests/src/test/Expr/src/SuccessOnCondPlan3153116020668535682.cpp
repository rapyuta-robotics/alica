#include <alica_tests/SuccessOnCondPlan3153116020668535682.h>
/*PROTECTED REGION ID(eph3153116020668535682) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SuccessOnCondPlan (3153116020668535682)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 173667041779969680)
//
// States:
//   - WaitForCondState (4012637487828402178)
//   - CondSuccessState (327448157837662747)
SuccessOnCondPlan3153116020668535682::SuccessOnCondPlan3153116020668535682(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3153116020668535682) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessOnCondPlan3153116020668535682::~SuccessOnCondPlan3153116020668535682()
{
    /*PROTECTED REGION ID(dcon3153116020668535682) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 173667041779969680
 */
std::shared_ptr<UtilityFunction> UtilityFunction3153116020668535682::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3153116020668535682) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3153116020668535682) ENABLED START*/
// Add additional options here
void SuccessOnCondPlan3153116020668535682::onInit()
{
    auto* wm = dynamic_cast<alicaTests::TestWorldModelNew*>(getWorldModel());
    auto* tc = wm->getTestContext();
    tc->resetAllTransitions(getName());
}
/*PROTECTED REGION END*/
} // namespace alica
