#include <alica_tests/SuccessOnCondWrapperB2869465844414224272.h>
/*PROTECTED REGION ID(eph2869465844414224272) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SuccessOnCondWrapperB (2869465844414224272)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4028079325935958114)
//
// States:
//   - SuccessOnCondState (3041287508452800918)
//   - WrapperBSuccessState (27419054733292598)
SuccessOnCondWrapperB2869465844414224272::SuccessOnCondWrapperB2869465844414224272(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2869465844414224272) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessOnCondWrapperB2869465844414224272::~SuccessOnCondWrapperB2869465844414224272()
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

/*PROTECTED REGION ID(methods2869465844414224272) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
