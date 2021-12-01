#include "HandleFailExplicitMaster1530004940652.h"
/*PROTECTED REGION ID(eph1530004940652) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

#include <memory>

namespace alica
{
// Plan:  HandleFailExplicitMaster (1530004940652)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1530004940654)
//
// States:
//   - NewState (1530004940653)
HandleFailExplicitMaster1530004940652::HandleFailExplicitMaster1530004940652(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1530004940652) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
HandleFailExplicitMaster1530004940652::~HandleFailExplicitMaster1530004940652()
{
    /*PROTECTED REGION ID(dcon1530004940652) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530004940654
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530004940652::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530004940652) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1530004940652) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
