#include <alica_tests/Configurations/ReadConfInPlantype1588061801734.h>
/*PROTECTED REGION ID(eph1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ReadConfInPlantype (1588061801734)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1588103719479)
//
// States:
//   - Default Name (1588103714226)
//   - ConfA (1588246134801)
//   - ConfB (1588246136647)
ReadConfInPlantype1588061801734::ReadConfInPlantype1588061801734(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1588061801734) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ReadConfInPlantype1588061801734::~ReadConfInPlantype1588061801734()
{
    /*PROTECTED REGION ID(dcon1588061801734) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588103719479
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588061801734::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061801734) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1588246141555)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> ConfA
 *
 * Precondition: 1588246141557 (1588246141557)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition1588246141557::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm)
{
    /*PROTECTED REGION ID(1588246141555) ENABLED START*/
    std::cout << "The PreCondition 1588246141557 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1588246144840)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> ConfB
 *
 * Precondition: 1588246144841 (1588246144841)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition1588246144841::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm)
{
    /*PROTECTED REGION ID(1588246144840) ENABLED START*/
    std::cout << "The PreCondition 1588246144841 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
