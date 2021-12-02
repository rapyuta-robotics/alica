#include "PlanFive1407153703092.h"
/*PROTECTED REGION ID(eph1407153703092) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanFive (1407153703092)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1407153717809)//   - AttackTask (1407153522080) (Entrypoint: 1407153972059)//   - DefendTask (1402488486725)
//   (Entrypoint: 1407153973706)//   - MidFieldTask (1407153536219) (Entrypoint: 1407153975075)
//
// States:
//   - DefaultState (1407153717808)
//   - DefendState (1407153985762)
//   - AttackState (1407153987910)
//   - MidFieldState (1407153989550)
PlanFive1407153703092::PlanFive1407153703092()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1407153703092) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanFive1407153703092::~PlanFive1407153703092()
{
    /*PROTECTED REGION ID(dcon1407153703092) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153717809
 * Task: AttackTask  -> EntryPoint-ID: 1407153972059
 * Task: DefendTask  -> EntryPoint-ID: 1407153973706
 * Task: MidFieldTask  -> EntryPoint-ID: 1407153975075
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153703092::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153703092) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407153703092) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
