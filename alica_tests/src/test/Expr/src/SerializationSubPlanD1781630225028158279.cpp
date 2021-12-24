#include "SerializationSubPlanD1781630225028158279.h"
/*PROTECTED REGION ID(eph1781630225028158279) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SerializationSubPlanD (1781630225028158279)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2209707708094369332)
//
// States:
//   - EntryState (4372755713553523771)
SerializationSubPlanD1781630225028158279::SerializationSubPlanD1781630225028158279(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1781630225028158279) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SerializationSubPlanD1781630225028158279::~SerializationSubPlanD1781630225028158279()
{
    /*PROTECTED REGION ID(dcon1781630225028158279) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool SerializationSubPlanD1781630225028158279::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext1781630225028158279) ENABLED START*/
    // Add additional options here
    return entryPointMap.begin()->second.insert(1).second;
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2209707708094369332
 */
std::shared_ptr<UtilityFunction> UtilityFunction1781630225028158279::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1781630225028158279) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1781630225028158279) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
