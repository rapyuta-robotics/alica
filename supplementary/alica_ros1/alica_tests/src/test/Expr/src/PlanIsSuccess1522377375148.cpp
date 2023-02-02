#include <alica_tests/PlanIsSuccess1522377375148.h>
/*PROTECTED REGION ID(eph1522377375148) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanIsSuccess (1522377375148)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1522377375150)
//
// States:
//   - Normal (1522377375149)
//   - Dummy (1522377929290)
//   - End (4341767214611490181)
PlanIsSuccess1522377375148::PlanIsSuccess1522377375148(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanIsSuccess1522377375148::~PlanIsSuccess1522377375148()
{
    /*PROTECTED REGION ID(dcon1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1522377375150
 */
std::shared_ptr<UtilityFunction> UtilityFunction1522377375148::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1522377375148) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/**
 * Transition: MISSING_NAME (1522377944058)
 *   - Comment:
 *   - Source2Dest: Normal --> Dummy
 *
 * Precondition: MISSING_NAME (1522377944921)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Normal:
 *   - SuccessNormal (2951008684180289642)
 */
bool PreCondition1522377944921::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1522377944058) ENABLED START*/
    std::cout << "The PreCondition 1522377944921 in Transition 'MISSING_NAME' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: MISSING_NAME (1522377945069)
 *   - Comment:
 *   - Source2Dest: Dummy --> Normal
 *
 * Precondition: MISSING_NAME (1522377946607)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Dummy:
 *   - SuccessDummy (3505111757300078074)
 */
bool PreCondition1522377946607::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1522377945069) ENABLED START*/
    std::cout << "The PreCondition 1522377946607 in Transition 'MISSING_NAME' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3694363277253985460 (3694363277253985460)
 *   - Comment:
 *   - Source2Dest: Dummy --> End
 *
 * Precondition: 2098555926520572428 (2098555926520572428)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Dummy:
 *   - SuccessDummy (3505111757300078074)
 */
bool PreCondition2098555926520572428::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(3694363277253985460) ENABLED START*/
    std::cout << "The PreCondition 2098555926520572428 in Transition '3694363277253985460' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1522377375148) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
