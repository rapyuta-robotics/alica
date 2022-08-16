#include <supplementary_tests/VariableHandling/Lvl11524452759599.h>
/*PROTECTED REGION ID(eph1524452759599) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Lvl1 (1524452759599)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1524452759601)
//
// States:
//   - NewState (1524452759600)
//   - BeforeTrans (1524453481856)
Lvl11524452759599::Lvl11524452759599(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1524452759599) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Lvl11524452759599::~Lvl11524452759599()
{
    /*PROTECTED REGION ID(dcon1524452759599) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): Lvl1 Runtime Condition, (Comment) :

/**
 * Available Vars:
 *	- L1A (1524453326397)
 *	- L1B (1524453331530)
 */
bool RunTimeCondition1524453470580::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1524453470580) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452759601
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452759599::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452759599) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1524452759599) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
