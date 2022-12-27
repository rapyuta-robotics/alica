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
std::shared_ptr<UtilityFunction> UtilityFunction1682631238618360548::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1682631238618360548) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 4079672912751513705 (4079672912751513705)
 *   - Comment:
 *   - Source2Dest: WaitState --> SucState
 *
 * Precondition: 1067314038887345208 (1067314038887345208)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in WaitState:
 */
bool PreCondition1067314038887345208::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(4079672912751513705) ENABLED START*/
    std::cout << "The PreCondition 1067314038887345208 in Transition '4079672912751513705' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1747408236004727286 (1747408236004727286)
 *   - Comment:
 *   - Source2Dest: EntryState --> WaitState
 *
 * Precondition: 597347780541336226 (597347780541336226)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 */
bool PreCondition597347780541336226::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1747408236004727286) ENABLED START*/
    std::cout << "The PreCondition 597347780541336226 in Transition '1747408236004727286' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1682631238618360548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
