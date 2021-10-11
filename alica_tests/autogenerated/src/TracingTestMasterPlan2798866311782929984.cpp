#include "TracingTestMasterPlan2798866311782929984.h"
/*PROTECTED REGION ID(eph2798866311782929984) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TracingTestMasterPlan (2798866311782929984)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1782718581605808075)
//
// States:
//   - EndTest (2317048855866339918)
//   - InitTest (1817522181941290968)
TracingTestMasterPlan2798866311782929984::TracingTestMasterPlan2798866311782929984()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con2798866311782929984) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TracingTestMasterPlan2798866311782929984::~TracingTestMasterPlan2798866311782929984()
{
    /*PROTECTED REGION ID(dcon2798866311782929984) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1782718581605808075
 */
std::shared_ptr<UtilityFunction> UtilityFunction2798866311782929984::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2798866311782929984) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 4591920804716698102 (4591920804716698102)
 *   - Comment:
 *   - Source2Dest: InitTest --> EndTest
 *
 * Precondition: 657847900438516106 (657847900438516106)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitTest:
 *   - TracingDisabledPlan (3148641312534759067)
 */
bool PreCondition657847900438516106::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(4591920804716698102) ENABLED START*/
    std::cout << "The PreCondition 657847900438516106 in Transition '4591920804716698102' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2798866311782929984) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
