#include <supplementary_tests/VariableHandling/VHMaster1524452721452.h>
/*PROTECTED REGION ID(eph1524452721452) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  VHMaster (1524452721452)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1524452721454)
//
// States:
//   - NewState (1524452721453)
VHMaster1524452721452::VHMaster1524452721452(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1524452721452) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
VHMaster1524452721452::~VHMaster1524452721452()
{
    /*PROTECTED REGION ID(dcon1524452721452) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): VHMaster Runtime Condition, (Comment) : Unrelated Condition

/**
 * Available Vars:
 *	- MA (1524463022262)
 *	- MB (1524463028066)
 */
bool RunTimeCondition1524463006078::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* worldModels)
{
    /*PROTECTED REGION ID(1524463006078) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452721454
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452721452::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452721452) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1524452721452) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
