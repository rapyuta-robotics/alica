#include "TracingDisabledPlan3148641312534759067.h"
/*PROTECTED REGION ID(eph3148641312534759067) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TracingDisabledPlan (3148641312534759067)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2870795844818415325)
//
// States:
//   - RunBehaviour (4222160850529504715)
TracingDisabledPlan3148641312534759067::TracingDisabledPlan3148641312534759067()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con3148641312534759067) ENABLED START*/
    // Add additional options here
    disableTracing();
    /*PROTECTED REGION END*/
}
TracingDisabledPlan3148641312534759067::~TracingDisabledPlan3148641312534759067()
{
    /*PROTECTED REGION ID(dcon3148641312534759067) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2870795844818415325
 */
std::shared_ptr<UtilityFunction> UtilityFunction3148641312534759067::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3148641312534759067) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3148641312534759067) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
