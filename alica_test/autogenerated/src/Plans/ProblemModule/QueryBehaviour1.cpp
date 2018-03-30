using namespace std;
#include "Plans/ProblemModule/QueryBehaviour1.h"

/*PROTECTED REGION ID(inccpp1479556104511) ENABLED START*/  // Add additional includes here
#include "SolverType.h"
/*PROTECTED REGION END*/
namespace alica {
/*PROTECTED REGION ID(staticVars1479556104511) ENABLED START*/  // initialise static variables here
vector<double> QueryBehaviour1::result;
/*PROTECTED REGION END*/
QueryBehaviour1::QueryBehaviour1() : DomainBehaviour("QueryBehaviour1") {
    /*PROTECTED REGION ID(con1479556104511) ENABLED START*/  // Add additional options here
    this->callCounter = 0;
    /*PROTECTED REGION END*/
}
QueryBehaviour1::~QueryBehaviour1() {
    /*PROTECTED REGION ID(dcon1479556104511) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
void QueryBehaviour1::run(void* msg) {
    /*PROTECTED REGION ID(run1479556104511) ENABLED START*/  // Add additional options here
    callCounter++;
    // cout << "QueryBehaviour1 was called " << callCounter << " times!" << endl;
    query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
    /*PROTECTED REGION END*/
}
void QueryBehaviour1::initialiseParameters() {
    /*PROTECTED REGION ID(initialiseParameters1479556104511) ENABLED START*/  // Add additional options here
    this->query = make_shared<alica::Query>(this->getRunningPlan()->getAlicaEngine());
    query->clearStaticVariables();
    query->addStaticVariable(getVariablesByName("QBX"));
    query->addStaticVariable(getVariablesByName("QBY"));
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479556104511) ENABLED START*/  // Add additional methods here
int QueryBehaviour1::getCallCounter() {
    return callCounter;
}
/*PROTECTED REGION END*/
} /* namespace alica */
