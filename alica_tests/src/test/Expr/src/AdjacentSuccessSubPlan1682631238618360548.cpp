#include <alica_tests/AdjacentSuccessSubPlan1682631238618360548.h>
/*PROTECTED REGION ID(eph1682631238618360548) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AdjacentSuccessSubPlan (1682631238618360548)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3470417373268048093)
//
// States:
//   - WaitState (235276470945876557)
//   - SucState (496520533178003845)
//   - EntryState (656998006978148289)
AdjacentSuccessSubPlan1682631238618360548::AdjacentSuccessSubPlan1682631238618360548(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1682631238618360548) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AdjacentSuccessSubPlan1682631238618360548::~AdjacentSuccessSubPlan1682631238618360548()
{
    /*PROTECTED REGION ID(dcon1682631238618360548) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3470417373268048093
 */

UtilityFunction1682631238618360548::UtilityFunction1682631238618360548(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1682631238618360548::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1682631238618360548) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1682631238618360548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
