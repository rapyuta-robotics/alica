#include "Configurations/ReadConfigurationPlan1588061334567.h"
/*PROTECTED REGION ID(eph1588061334567) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ReadConfigurationPlan (1588061334567)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1588069183324)
//
// States:
//   - DecisionState (1588069177860)
//   - StateA (1588069261047)
//   - StateB (1588069265377)
ReadConfigurationPlan1588061334567::ReadConfigurationPlan1588061334567(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1588061334567) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ReadConfigurationPlan1588061334567::~ReadConfigurationPlan1588061334567()
{
    /*PROTECTED REGION ID(dcon1588061334567) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588069183324
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588061334567::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061334567) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1588069612659)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: DecisionState --> StateA
 *
 * Precondition: 1588069612661 (1588069612661)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DecisionState:
 */
bool PreCondition1588069612661::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1588069612659) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("1") == 0;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1588069615552)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: DecisionState --> StateB
 *
 * Precondition: 1588069615553 (1588069615553)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DecisionState:
 */
bool PreCondition1588069615553::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1588069615552) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("2") == 0;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1588061334567) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
