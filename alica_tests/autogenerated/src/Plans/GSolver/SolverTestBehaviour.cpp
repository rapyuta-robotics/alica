using namespace std;
#include "Plans/GSolver/SolverTestBehaviour.h"

/*PROTECTED REGION ID(inccpp1417424455986) ENABLED START*/  // Add additional includes here

#include <CGSolver.h>

/*PROTECTED REGION END*/
namespace alica {
/*PROTECTED REGION ID(staticVars1417424455986) ENABLED START*/  // initialise static variables here
std::vector<double> SolverTestBehaviour::result;
/*PROTECTED REGION END*/
SolverTestBehaviour::SolverTestBehaviour()
        : DomainBehaviour("SolverTestBehaviour") {
    /*PROTECTED REGION ID(con1417424455986) ENABLED START*/  // Add additional options here
    callCounter = 0;
    /*PROTECTED REGION END*/
}
SolverTestBehaviour::~SolverTestBehaviour() {
    /*PROTECTED REGION ID(dcon1417424455986) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
void SolverTestBehaviour::run(void* msg) {
    /*PROTECTED REGION ID(run1417424455986) ENABLED START*/  // Add additional options here
    callCounter++;
    // std::cout << "SolverTestBehaviour was called " << callCounter << " times!" << std::endl;

    _query.getSolution<reasoner::CGSolver,double>(runningPlan, result);
    /*PROTECTED REGION END*/
}
void SolverTestBehaviour::initialiseParameters() {
    /*PROTECTED REGION ID(initialiseParameters1417424455986) ENABLED START*/  // Add additional options here
    _query.clearStaticVariables();
    _query.addStaticVariable(getVariableByName("X"));
    _query.addStaticVariable(getVariableByName("Y"));
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1417424455986) ENABLED START*/  // Add additional methods here
int SolverTestBehaviour::getCallCounter() {
    return callCounter;
}
/*PROTECTED REGION END*/
} /* namespace alica */
