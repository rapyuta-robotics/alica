#include "Tackle1402489318663.h"
/*PROTECTED REGION ID(eph1402489318663) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Tackle (1402489318663)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402489329142)
//
// States:
//   - AttackOpp (1402489329141)
Tackle1402489318663::Tackle1402489318663()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1402489318663) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Tackle1402489318663::~Tackle1402489318663()
{
    /*PROTECTED REGION ID(dcon1402489318663) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void Tackle1402489318663::run(void* msg)
{
    /*PROTECTED REGION ID(runTackle1402489318663) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402489329142
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402489318663::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402489318663) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402489318663) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
