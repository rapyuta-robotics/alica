#include <alica_tests/FailurePlan631515556091266493.h>
/*PROTECTED REGION ID(eph631515556091266493) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  FailurePlan (631515556091266493)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4488468250406966071)
//
// States:
//   - Init (1171453089016322268)
//   - Fail (3487518754011112127)
//   - Failed (3748960977005112327)
FailurePlan631515556091266493::FailurePlan631515556091266493(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con631515556091266493) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
FailurePlan631515556091266493::~FailurePlan631515556091266493()
{
    /*PROTECTED REGION ID(dcon631515556091266493) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4488468250406966071
 */
std::shared_ptr<UtilityFunction> UtilityFunction631515556091266493::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(631515556091266493) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods631515556091266493) ENABLED START*/
// Add additional options here
void FailurePlan631515556091266493::onInit()
{
    auto* worldModel = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    worldModel->failurePlanInitCalled();
}
/*PROTECTED REGION END*/
} // namespace alica