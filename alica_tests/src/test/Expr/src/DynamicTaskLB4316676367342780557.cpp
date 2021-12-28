#include "DynamicTaskLB4316676367342780557.h"
/*PROTECTED REGION ID(eph4316676367342780557) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskLB (4316676367342780557)
//
// Tasks:
//   - DynamicTask (1163169622598227531) (Entrypoint: 1022894855310263692)
//
// States:
//   - LC (2765772942388464345)
DynamicTaskLB4316676367342780557::DynamicTaskLB4316676367342780557(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con4316676367342780557) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskLB4316676367342780557::~DynamicTaskLB4316676367342780557()
{
    /*PROTECTED REGION ID(dcon4316676367342780557) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool DynamicTaskLB4316676367342780557::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext4316676367342780557) ENABLED START*/
    // Add additional options here
    return entryPointMap.begin()->second.insert(3).second;
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTask  -> EntryPoint-ID: 1022894855310263692
 */
std::shared_ptr<UtilityFunction> UtilityFunction4316676367342780557::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4316676367342780557) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4316676367342780557) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
