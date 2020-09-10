#include "ProblemModule/QueryBehaviour1.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1479556104511) ENABLED START*/
// Add additional includes here
#include <constraintsolver/CGSolver.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1479556104511) ENABLED START*/
// initialise static variables here
std::vector<double> QueryBehaviour1::result;
/*PROTECTED REGION END*/

QueryBehaviour1::QueryBehaviour1()
        : DomainBehaviour("QueryBehaviour1")
{
    /*PROTECTED REGION ID(con1479556104511) ENABLED START*/
    // Add additional options here
    this->callCounter = 0;
    /*PROTECTED REGION END*/
}
QueryBehaviour1::~QueryBehaviour1()
{
    /*PROTECTED REGION ID(dcon1479556104511) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void QueryBehaviour1::run(void* msg)
{
    /*PROTECTED REGION ID(run1479556104511) ENABLED START*/
    // Add additional options here
    callCounter++;
    // cout << "QueryBehaviour1 was called " << callCounter << " times!" << endl;

    std::lock_guard<std::mutex> guard(queryMutex);
    if (!stopQuerying) {
        query->getSolution<reasoner::CGSolver, double>(getPlanContext(), result);
    }
    /*PROTECTED REGION END*/
}
void QueryBehaviour1::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1479556104511) ENABLED START*/
    // Add additional options here
    this->query = make_shared<alica::Query>();
    query->clearStaticVariables();
    query->addStaticVariable(getVariable("QBX"));
    query->addStaticVariable(getVariable("QBY"));
    stopQuerying = false;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1479556104511) ENABLED START*/
// Add additional methods here
    void QueryBehaviour1::stopQueries()
    {
        std::lock_guard<std::mutex> guard(queryMutex);
        stopQuerying = true;
    }

int QueryBehaviour1::getCallCounter()
{
    return callCounter;
}
/*PROTECTED REGION END*/

} /* namespace alica */
