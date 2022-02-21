#include "DummyImplementation4126421719858579722.h"
/*PROTECTED REGION ID(eph4126421719858579722) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DummyImplementation (4126421719858579722)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2593906152181871206)
//
// States:
//   - DummySuccess (1823707990536087965)
//   - DummyState (400856976447099508)
DummyImplementation4126421719858579722::DummyImplementation4126421719858579722(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con4126421719858579722) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DummyImplementation4126421719858579722::~DummyImplementation4126421719858579722()
{
    /*PROTECTED REGION ID(dcon4126421719858579722) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2593906152181871206
 */
std::shared_ptr<UtilityFunction> UtilityFunction4126421719858579722::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4126421719858579722) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 293344705861516112 (293344705861516112)
 *   - Comment:
 *   - Source2Dest: DummyState --> DummySuccess
 *
 * Precondition: 3469760593538210700 (3469760593538210700)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DummyState:
 */
bool PreCondition3469760593538210700::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(293344705861516112) ENABLED START*/
    std::cout << "The PreCondition 3469760593538210700 in Transition '293344705861516112' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4126421719858579722) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
