#include "DynamicTaskLA3337489358878214836.h"
/*PROTECTED REGION ID(eph3337489358878214836) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskLA (3337489358878214836)
//
// Tasks:
//   - DynamicTask (1163169622598227531) (Entrypoint: 1666138843382003218)
//
// States:
//   - LB (1633421497783210879)
DynamicTaskLA3337489358878214836::DynamicTaskLA3337489358878214836(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con3337489358878214836) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskLA3337489358878214836::~DynamicTaskLA3337489358878214836()
{
    /*PROTECTED REGION ID(dcon3337489358878214836) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool DynamicTaskLA3337489358878214836::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext3337489358878214836) ENABLED START*/
    // Add additional options here
    return entryPointMap.begin()->second.insert(2).second;
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTask  -> EntryPoint-ID: 1666138843382003218
 */
std::shared_ptr<UtilityFunction> UtilityFunction3337489358878214836::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3337489358878214836) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3337489358878214836) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
