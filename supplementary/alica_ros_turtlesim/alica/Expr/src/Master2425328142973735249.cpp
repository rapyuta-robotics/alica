#include <alica/Master2425328142973735249.h>
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
//   - Move (2405597980801916441)
//   - Init (3997532517592149463)
Master2425328142973735249::Master2425328142973735249(PlanContext& context)
        : DomainPlan(context)
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

UtilityFunction2425328142973735249::UtilityFunction2425328142973735249(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction2425328142973735249::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2425328142973735249) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2425328142973735249) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
