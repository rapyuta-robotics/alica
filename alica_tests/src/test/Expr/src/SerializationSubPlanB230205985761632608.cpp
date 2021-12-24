#include "SerializationSubPlanB230205985761632608.h"
/*PROTECTED REGION ID(eph230205985761632608) ENABLED START*/
// Add additional options here
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SerializationSubPlanB (230205985761632608)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4537575568058788140)
//
// States:
//   - EntryState (1137921287745324120)
SerializationSubPlanB230205985761632608::SerializationSubPlanB230205985761632608(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con230205985761632608) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SerializationSubPlanB230205985761632608::~SerializationSubPlanB230205985761632608()
{
    /*PROTECTED REGION ID(dcon230205985761632608) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4537575568058788140
 */
std::shared_ptr<UtilityFunction> UtilityFunction230205985761632608::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(230205985761632608) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods230205985761632608) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
