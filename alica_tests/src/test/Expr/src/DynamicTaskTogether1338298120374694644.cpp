#include "DynamicTaskTogether1338298120374694644.h"
/*PROTECTED REGION ID(eph1338298120374694644) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskTogether (1338298120374694644)
//
// Tasks:
//   - DynamicTaskForDefender (4028411332434222682) (Entrypoint: 2665027307523422046)//   - DynamicTaskForAttacker (4026821563126910189) (Entrypoint:
//   2633712961224790694)
//
// States:
//   - 2564904534754645793 (2564904534754645793)
//   - 2362235348110947949 (2362235348110947949)
DynamicTaskTogether1338298120374694644::DynamicTaskTogether1338298120374694644(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1338298120374694644) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskTogether1338298120374694644::~DynamicTaskTogether1338298120374694644()
{
    /*PROTECTED REGION ID(dcon1338298120374694644) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool DynamicTaskTogether1338298120374694644::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext1338298120374694644) ENABLED START*/
    // Add additional options here
    bool ret = entryPointMap.begin()->second.insert(11).second;
    ret |= std::next(entryPointMap.begin())->second.insert(22).second;
    return ret;
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTaskForDefender  -> EntryPoint-ID: 2665027307523422046
 * Task: DynamicTaskForAttacker  -> EntryPoint-ID: 2633712961224790694
 */
std::shared_ptr<UtilityFunction> UtilityFunction1338298120374694644::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1338298120374694644) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1338298120374694644) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
