#include <alica_tests/SuccessOnInitPlan1863216812678266511.h>
/*PROTECTED REGION ID(eph1863216812678266511) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SuccessOnInitPlan (1863216812678266511)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3588095671831141137)
//
// States:
//   - SuccessOnInitSuccessState (66024355821713834)
//   - DummyInitState (250474402398721721)
SuccessOnInitPlan1863216812678266511::SuccessOnInitPlan1863216812678266511(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1863216812678266511) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessOnInitPlan1863216812678266511::~SuccessOnInitPlan1863216812678266511()
{
    /*PROTECTED REGION ID(dcon1863216812678266511) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3588095671831141137
 */
std::shared_ptr<UtilityFunction> UtilityFunction1863216812678266511::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1863216812678266511) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1863216812678266511) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
