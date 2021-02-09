#include "Authority/AuthorityTest1414403413451.h"
/*PROTECTED REGION ID(eph1414403413451) ENABLED START*/
// Add additional using directives here
#include "DummyTestSummand.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:AuthorityTest1414403413451
AuthorityTest1414403413451::AuthorityTest1414403413451()
        : DomainPlan("AuthorityTest1414403413451")
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
} // namespace alica
