#include "OtherPlan1418042819203.h"
/*PROTECTED REGION ID(eph1418042819203) ENABLED START*/
// Add additional using directives here
#include "engine/USummand.h"
#include <alica_tests/DistXContourTest.h>
#include <alica_tests/TestConstantValueSummand.h>
/*PROTECTED REGION END*/

#include <memory>

namespace alica
{
// Plan:  OtherPlan (1418042819203)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042819206)
//
// States:
//   - OtherPlanTest (1418042819204)
OtherPlan1418042819203::OtherPlan1418042819203(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1418042819203) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
OtherPlan1418042819203::~OtherPlan1418042819203()
{
    /*PROTECTED REGION ID(dcon1418042819203) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042819206
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042819203::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042819203) ENABLED START*/

    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 0.2));
    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042819203) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
