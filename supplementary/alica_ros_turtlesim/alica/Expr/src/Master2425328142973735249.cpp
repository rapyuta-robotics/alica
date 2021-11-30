#include "Master2425328142973735249.h"
/*PROTECTED REGION ID(eph2425328142973735249) ENABLED START*/
// Add additional options here
#include <alica_ros_turtlesim/world_model.hpp>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Master (2425328142973735249)
//
// Tasks:
//   - DefaultTask (3310236980587704776) (Entrypoint: 2741715629576575326)
//
// States:
//   - Init (3997532517592149463)
//   - Move (2405597980801916441)
Master2425328142973735249::Master2425328142973735249(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2425328142973735249) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Master2425328142973735249::~Master2425328142973735249()
{
    /*PROTECTED REGION ID(dcon2425328142973735249) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2741715629576575326
 */
std::shared_ptr<UtilityFunction> UtilityFunction2425328142973735249::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2425328142973735249) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3486027875296378577 (3486027875296378577)
 *   - Comment:
 *   - Source2Dest: Init --> Move
 *
 * Precondition: Init2Move (1597434482701133956)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 *   - Go2RandomPosition (4085572422059465423)
 */
bool PreCondition1597434482701133956::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3486027875296378577) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}
/**
 * Transition: 635844345274619238 (635844345274619238)
 *   - Comment:
 *   - Source2Dest: Move --> Init
 *
 * Precondition: Move2Init (1136497454350831106)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Move:
 *   - Move (1889749086610694100)
 */
bool PreCondition1136497454350831106::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(635844345274619238) ENABLED START*/
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2425328142973735249) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
