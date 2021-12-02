#include "PlanPoolTestSubPlan432995127772554364.h"
/*PROTECTED REGION ID(eph432995127772554364) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanPoolTestSubPlan (432995127772554364)
//
// Tasks:
//   - Attack (1222613952469) (Entrypoint: 3382470863218497440)
//
// States:
//   - 297850764330117621 (297850764330117621)
PlanPoolTestSubPlan432995127772554364::PlanPoolTestSubPlan432995127772554364()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con432995127772554364) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanPoolTestSubPlan432995127772554364::~PlanPoolTestSubPlan432995127772554364()
{
    /*PROTECTED REGION ID(dcon432995127772554364) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: Attack  -> EntryPoint-ID: 3382470863218497440
 */
std::shared_ptr<UtilityFunction> UtilityFunction432995127772554364::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(432995127772554364) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods432995127772554364) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
