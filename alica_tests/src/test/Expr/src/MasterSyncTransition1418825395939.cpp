#include <alica_tests/MasterSyncTransition1418825395939.h>
/*PROTECTED REGION ID(eph1418825395939) ENABLED START*/
// Add additional using directives here
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MasterSyncTransition (1418825395939)
//
// Tasks:
//   - AttackTask (1407153522080) (Entrypoint: 1418825395941)//   - DefaultTask (1225112227903) (Entrypoint: 1418825402617)
//
// States:
//   - FirstTaskFirstState (1418825395940)
//   - SecondTaskFirstState (1418825404963)
//   - FirstTaskSecondState (1418825409988)
//   - SecondTaskSecondState (1418825411686)
MasterSyncTransition1418825395939::MasterSyncTransition1418825395939(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1418825395939) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterSyncTransition1418825395939::~MasterSyncTransition1418825395939()
{
    /*PROTECTED REGION ID(dcon1418825395939) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: AttackTask  -> EntryPoint-ID: 1418825395941
 * Task: DefaultTask  -> EntryPoint-ID: 1418825402617
 */

UtilityFunction1418825395939::UtilityFunction1418825395939(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1418825395939::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418825395939) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418825395939) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
