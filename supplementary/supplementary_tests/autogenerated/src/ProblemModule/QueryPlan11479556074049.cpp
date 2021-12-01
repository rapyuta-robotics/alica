#include "ProblemModule/QueryPlan11479556074049.h"
/*PROTECTED REGION ID(eph1479556074049) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

#include <memory>

namespace alica
{
// Plan:  QueryPlan1 (1479556074049)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1479556074051)
//
// States:
//   - QueryState1 (1479556074050)
QueryPlan11479556074049::QueryPlan11479556074049(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1479556074049) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
QueryPlan11479556074049::~QueryPlan11479556074049()
{
    /*PROTECTED REGION ID(dcon1479556074049) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- QP1X (1479556220234)
 *	- QP1Y (1479556572534)
 */
bool RunTimeCondition1479556084493::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1479556084493) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479556074051
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479556074049::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479556074049) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479556074049) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
