#include "SerializationSubPlanA1433931143598606082.h"
/*PROTECTED REGION ID(eph1433931143598606082) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SerializationSubPlanA (1433931143598606082)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3603519261566578758)
//
// States:
//   - EntryState (1059656669948994292)
SerializationSubPlanA1433931143598606082::SerializationSubPlanA1433931143598606082(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1433931143598606082) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SerializationSubPlanA1433931143598606082::~SerializationSubPlanA1433931143598606082()
{
    /*PROTECTED REGION ID(dcon1433931143598606082) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3603519261566578758
 */
std::shared_ptr<UtilityFunction> UtilityFunction1433931143598606082::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1433931143598606082) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1433931143598606082) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
