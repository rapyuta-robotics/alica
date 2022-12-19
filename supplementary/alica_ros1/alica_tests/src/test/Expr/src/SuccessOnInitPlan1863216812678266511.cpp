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

/**
 * Transition: 4061633193878470475 (4061633193878470475)
 *   - Comment:
 *   - Source2Dest: DummyInitState --> SuccessOnInitSuccessState
 *
 * Precondition: 4197030928062612573 (4197030928062612573)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DummyInitState:
 */
bool PreCondition4197030928062612573::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(4061633193878470475) ENABLED START*/
    std::cout << "The PreCondition 4197030928062612573 in Transition '4061633193878470475' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1863216812678266511) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
