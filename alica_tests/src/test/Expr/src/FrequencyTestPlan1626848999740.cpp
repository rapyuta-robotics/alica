#include "FrequencyTestPlan1626848999740.h"
/*PROTECTED REGION ID(eph1626848999740) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  FrequencyTestPlan (1626848999740)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1626849024805)
//
// States:
//   - Default Name (1626849027475)
FrequencyTestPlan1626848999740::FrequencyTestPlan1626848999740(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1626848999740) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
FrequencyTestPlan1626848999740::~FrequencyTestPlan1626848999740()
{
    /*PROTECTED REGION ID(dcon1626848999740) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1626849024805
 */
std::shared_ptr<UtilityFunction> UtilityFunction1626848999740::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1626848999740) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1626848999740) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
