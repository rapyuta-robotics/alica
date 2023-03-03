#include "engine/USummand.h"
#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/plans/Authority/AuthorityTest.h>

namespace alica
{
AuthorityTest::AuthorityTest(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> AuthorityTestUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.1, 0.1, plan);
    DummyTestSummand* us = new DummyTestSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(1414403522424));
    us->addEntryPoint(plan->getEntryPointByID(1414403429951));

    function->editUtilSummands().emplace_back(us);

    return function;
}

} // namespace alica
