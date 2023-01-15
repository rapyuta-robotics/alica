#include <alica_tests/SuccessOnCondWrapperBPlan2869465844414224272.h>
/*PROTECTED REGION ID(eph2869465844414224272) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SuccessOnCondWrapperBPlan (2869465844414224272)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4028079325935958114)
//
// States:
//   - WrapperBSuccessState (27419054733292598)
//   - SuccessOnCondState (3041287508452800918)
SuccessOnCondWrapperBPlan2869465844414224272::SuccessOnCondWrapperBPlan2869465844414224272(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2869465844414224272) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessOnCondWrapperBPlan2869465844414224272::~SuccessOnCondWrapperBPlan2869465844414224272()
{
    /*PROTECTED REGION ID(dcon2869465844414224272) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4028079325935958114
 */
std::shared_ptr<UtilityFunction> UtilityFunction2869465844414224272::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2869465844414224272) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 796950042414573400 (796950042414573400)
 *   - Comment:
 *   - Source2Dest: SuccessOnCondState --> WrapperBSuccessState
 *
 * Precondition: 914907830776317719 (914907830776317719)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in SuccessOnCondState:
 *   - SuccessOnCondPlan (3153116020668535682)
 */
bool PreCondition914907830776317719::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(796950042414573400) ENABLED START*/
    std::cout << "The PreCondition 914907830776317719 in Transition '796950042414573400' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2869465844414224272) ENABLED START*/
// Add additional options here
void SuccessOnCondWrapperBPlan2869465844414224272::onInit() {}
/*PROTECTED REGION END*/
} // namespace alica
