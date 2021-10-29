#include "AdjacentSuccessMasterPlan3254486013443203397.h"
/*PROTECTED REGION ID(eph3254486013443203397) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AdjacentSuccessMasterPlan (3254486013443203397)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 177437342277134781)
//
// States:
//   - EntryState (338845808462999166)
//   - SecondState (1114306208475690481)
AdjacentSuccessMasterPlan3254486013443203397::AdjacentSuccessMasterPlan3254486013443203397()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con3254486013443203397) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AdjacentSuccessMasterPlan3254486013443203397::~AdjacentSuccessMasterPlan3254486013443203397()
{
    /*PROTECTED REGION ID(dcon3254486013443203397) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 177437342277134781
 */
std::shared_ptr<UtilityFunction> UtilityFunction3254486013443203397::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3254486013443203397) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3345031375302716643 (3345031375302716643)
 *   - Comment:
 *   - Source2Dest: EntryState --> SecondState
 *
 * Precondition: 807250359520655888 (807250359520655888)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 *   - AdjacentSuccessSubPlan (1682631238618360548)
 */
bool PreCondition807250359520655888::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(3345031375302716643) ENABLED START*/
    if (alicaTests::TestWorldModel::getOne()->isTransitionCondition3345031375302716643()) {
        std::cerr << "doTransitionMasterPlan FromEntryToSecond" << std::endl;
        alicaTests::TestWorldModel::getOne()->setTransitionCondition3345031375302716643(false);
        alicaTests::TestWorldModel::getOne()->setTransitionCondition1914245867924544479(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 1914245867924544479 (1914245867924544479)
 *   - Comment:
 *   - Source2Dest: SecondState --> EntryState
 *
 * Precondition: 289358204208851392 (289358204208851392)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in SecondState:
 *   - AdjacentSuccessSubPlan (1682631238618360548)
 */
bool PreCondition289358204208851392::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1914245867924544479) ENABLED START*/
    if (alicaTests::TestWorldModel::getOne()->isTransitionCondition1914245867924544479()) {
        std::cerr << "doTransition MasterPlan FromSecondToEntry" << std::endl;
        alicaTests::TestWorldModel::getOne()->setTransitionCondition3345031375302716643(false);
        alicaTests::TestWorldModel::getOne()->setTransitionCondition1914245867924544479(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3254486013443203397) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
