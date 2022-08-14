#include <alica_tests/RuntimeConditionPlan1418042806575.h>
/*PROTECTED REGION ID(eph1418042806575) ENABLED START*/
// Add additional using directives here
#include "engine/USummand.h"
#include <alica_tests/DistXContourTest.h>
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  RuntimeConditionPlan (1418042806575)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042806577)
//
// States:
//   - RuntimeConditionTest (1418042806576)
RuntimeConditionPlan1418042806575::RuntimeConditionPlan1418042806575(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1418042806575) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
RuntimeConditionPlan1418042806575::~RuntimeConditionPlan1418042806575()
{
    /*PROTECTED REGION ID(dcon1418042806575) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1418042967134::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1418042967134) ENABLED START*/
    auto* worldModel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldModel->isRuntimeCondition1418042967134();
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042806577
 */

UtilityFunction1418042806575::UtilityFunction1418042806575(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1418042806575::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042806575) ENABLED START*/
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan, _logger);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042806575) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
