#include "Authority/AuthorityTest1414403413451.h"
/*PROTECTED REGION ID(eph1414403413451) ENABLED START*/
// Add additional using directives here
#include "engine/USummand.h"
#include <alica_tests/DistXContourTest.h>
#include <alica_tests/DummyTestSummand.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AuthorityTest (1414403413451)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1414403429951)//   - AttackTask (1407153522080) (Entrypoint: 1414403522424)
//
// States:
//   - UpperState (1414403429950)
//   - LowerState (1414403553717)
AuthorityTest1414403413451::AuthorityTest1414403413451()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1414403413451) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AuthorityTest1414403413451::~AuthorityTest1414403413451()
{
    /*PROTECTED REGION ID(dcon1414403413451) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void AuthorityTest1414403413451::run(void* msg)
{
    /*PROTECTED REGION ID(runAuthorityTest1414403413451) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414403429951
 * Task: AttackTask  -> EntryPoint-ID: 1414403522424
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414403413451::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414403413451) ENABLED START*/
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.1, 0.1, plan);
    DummyTestSummand* us = new DummyTestSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(1414403522424));
    us->addEntryPoint(plan->getEntryPointByID(1414403429951));

    function->editUtilSummands().emplace_back(us);

    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1414403413451) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
