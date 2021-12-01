#include "PlanTwo1407153645238.h"
/*PROTECTED REGION ID(eph1407153645238) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

#include <memory>

namespace alica
{
// Plan:  PlanTwo (1407153645238)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1407153656782)//   - AttackTask (1407153522080) (Entrypoint: 1407153821287)//   - DefendTask (1402488486725)
//   (Entrypoint: 1407153842648)
//
// States:
//   - DefaultState (1407153656781)
//   - AttackState (1407153860891)
//   - DefendState (1407153869754)
PlanTwo1407153645238::PlanTwo1407153645238(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1407153645238) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanTwo1407153645238::~PlanTwo1407153645238()
{
    /*PROTECTED REGION ID(dcon1407153645238) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153656782
 * Task: AttackTask  -> EntryPoint-ID: 1407153821287
 * Task: DefendTask  -> EntryPoint-ID: 1407153842648
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153645238::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153645238) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407153645238) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
