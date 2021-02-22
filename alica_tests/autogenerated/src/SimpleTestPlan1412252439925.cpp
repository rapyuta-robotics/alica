#include "SimpleTestPlan1412252439925.h"
/*PROTECTED REGION ID(eph1412252439925) ENABLED START*/
// Add additional using directives here
#include "CounterClass.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SimpleTestPlan1412252439925
SimpleTestPlan1412252439925::SimpleTestPlan1412252439925()
        : DomainPlan("SimpleTestPlan1412252439925")
{
    /*PROTECTED REGION ID(con1412252439925) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SimpleTestPlan1412252439925::~SimpleTestPlan1412252439925()
{
    /*PROTECTED REGION ID(dcon1412252439925) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
// Check of PreCondition - (Name): NewPreCondition, (ConditionString):  , (Comment) :

/**
 * Available Vars:
 */
bool PreCondition1412781707952::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1412781707952) ENABLED START*/
    //--> "PreCondition:1412781707952  not implemented";
    return true;
    /*PROTECTED REGION END*/
}
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1412781693884::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1412781693884) ENABLED START*/
    CounterClass::called++;
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1412252439927
 */
std::shared_ptr<UtilityFunction> UtilityFunction1412252439925::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1412252439925) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1412761926856, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - MidFieldStandard (1402488696205)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1412252439927)
 *
 * States in plan:
 *   - TestState1 (1412252439926)
 *   - TestState2 (1412761855746)
 *
 * Variables of precondition:
 */
bool PreCondition1412761926856::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1412761925032) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1412252439925) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
