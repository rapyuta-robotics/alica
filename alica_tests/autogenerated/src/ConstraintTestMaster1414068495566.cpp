#include "ConstraintTestMaster1414068495566.h"
/*PROTECTED REGION ID(eph1414068495566) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ConstraintTestMaster1414068495566
ConstraintTestMaster1414068495566::ConstraintTestMaster1414068495566()
        : DomainPlan("ConstraintTestMaster1414068495566")
{
    /*PROTECTED REGION ID(con1414068495566) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ConstraintTestMaster1414068495566::~ConstraintTestMaster1414068495566()
{
    /*PROTECTED REGION ID(dcon1414068495566) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414068495568
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414068495566::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414068495566) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1414068495566) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
