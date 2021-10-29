#include "AdjacentSuccessSubPlan1682631238618360548.h"
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
//   - 1181007688948643441 (1181007688948643441)
//   - 496520533178003845 (496520533178003845)
//   - 656998006978148289 (656998006978148289)
AdjacentSuccessSubPlan1682631238618360548::AdjacentSuccessSubPlan1682631238618360548()
        : DomainPlan()
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
 * Transition: 1390829819585906015 (1390829819585906015)
 *   - Comment:
 *   - Source2Dest: 1181007688948643441 --> 496520533178003845
 *
 * Precondition: 3875618235052823378 (3875618235052823378)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 1181007688948643441:
 *   - SuccessSpam (1522377401286)
 */
bool PreCondition3875618235052823378::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1390829819585906015) ENABLED START*/
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (child->getStatus() == alica::PlanStatus::Success) {
            std::cerr << "succeeding in subplan" << std::endl;
            return true;
        }
    }
    return false;
    // if (rp->isBehaviour()) {
    //     std::cerr << "isBehaviour" << std::endl;
    //     if (rp->getStatus() == alica::PlanStatus::Success) {
    //         std::cerr << "doTransition subPlan" << std::endl;
    //     }
    //     return rp->getStatus() == alica::PlanStatus::Success;
    // }
    // return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3143778092687974738 (3143778092687974738)
 *   - Comment:
 *   - Source2Dest: 656998006978148289 --> 1181007688948643441
 *
 * Precondition: 3441061963559991094 (3441061963559991094)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 656998006978148289:
 */
bool PreCondition3441061963559991094::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(3143778092687974738) ENABLED START*/
    if (alicaTests::TestWorldModel::getOne()->isTransitionCondition3143778092687974738()) {
        alicaTests::TestWorldModel::getOne()->setTransitionCondition3143778092687974738(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1682631238618360548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
