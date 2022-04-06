#include "TaskInstantiationIntegrationTestMaster4603312216886200747.h"
/*PROTECTED REGION ID(eph4603312216886200747) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TaskInstantiationIntegrationTestMaster (4603312216886200747)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1269233481168214500)
//
// States:
//   - RunTest (824745231464311542)
TaskInstantiationIntegrationTestMaster4603312216886200747::TaskInstantiationIntegrationTestMaster4603312216886200747(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con4603312216886200747) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TaskInstantiationIntegrationTestMaster4603312216886200747::~TaskInstantiationIntegrationTestMaster4603312216886200747()
{
    /*PROTECTED REGION ID(dcon4603312216886200747) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1269233481168214500
 */
std::shared_ptr<UtilityFunction> UtilityFunction4603312216886200747::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4603312216886200747) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4603312216886200747) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
