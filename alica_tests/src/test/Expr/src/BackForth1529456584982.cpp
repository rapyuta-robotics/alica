#include "BackForth1529456584982.h"
/*PROTECTED REGION ID(eph1529456584982) ENABLED START*/
// Add additional using directives here
#include <alica_tests/SimpleSwitches.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BackForth (1529456584982)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1529456584984)
//
// States:
//   - First (1529456584983)
//   - Second (1529456591410)
BackForth1529456584982::BackForth1529456584982(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1529456584982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BackForth1529456584982::~BackForth1529456584982()
{
    /*PROTECTED REGION ID(dcon1529456584982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1529456584984
 */
std::shared_ptr<UtilityFunction> UtilityFunction1529456584982::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1529456584982) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/**
 * Transition: MISSING_NAME (1529456609989)
 *   - Comment: Forth
 *   - Source2Dest: First --> Second
 *
 * Precondition: MISSING_NAME (1529456610697)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in First:
 *   - CountIndefinitely (1529456643148)
 */
bool PreCondition1529456610697::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1529456609989) ENABLED START*/
    return SimpleSwitches::isSet(0);
    /*PROTECTED REGION END*/
}

/**
 * Transition: MISSING_NAME (1529456610905)
 *   - Comment: Back
 *   - Source2Dest: Second --> First
 *
 * Precondition: MISSING_NAME (1529456611916)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Second:
 *   - CountIndefinitely (1529456643148)
 */
bool PreCondition1529456611916::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1529456610905) ENABLED START*/
    return SimpleSwitches::isSet(1);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1529456584982) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
