#include "EmptyPlan984284423749038756.h"
/*PROTECTED REGION ID(eph984284423749038756) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  EmptyPlan (984284423749038756)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 816950277819536587)
//
// States:
//   - 3164334534532883889 (3164334534532883889)
EmptyPlan984284423749038756::EmptyPlan984284423749038756(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con984284423749038756) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
EmptyPlan984284423749038756::~EmptyPlan984284423749038756()
{
    /*PROTECTED REGION ID(dcon984284423749038756) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 816950277819536587
 */
std::shared_ptr<UtilityFunction> UtilityFunction984284423749038756::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(984284423749038756) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods984284423749038756) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
