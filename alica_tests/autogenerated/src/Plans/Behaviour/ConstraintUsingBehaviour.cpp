using namespace std;
#include "Plans/Behaviour/ConstraintUsingBehaviour.h"

/*PROTECTED REGION ID(inccpp1414068597716) ENABLED START*/  // Add additional includes here
#include <iostream>
/*PROTECTED REGION END*/
namespace alica {
/*PROTECTED REGION ID(staticVars1414068597716) ENABLED START*/  // initialise static variables here
vector<string> ConstraintUsingBehaviour::result;
/*PROTECTED REGION END*/
ConstraintUsingBehaviour::ConstraintUsingBehaviour()
        : DomainBehaviour("ConstraintUsingBehaviour") {
    /*PROTECTED REGION ID(con1414068597716) ENABLED START*/  // Add additional options here
    this->callCounter = 0;
    /*PROTECTED REGION END*/
}
ConstraintUsingBehaviour::~ConstraintUsingBehaviour() {
    /*PROTECTED REGION ID(dcon1414068597716) ENABLED START*/  // Add additional options here
    /*PROTECTED REGION END*/
}
void ConstraintUsingBehaviour::run(void* msg) {
    /*PROTECTED REGION ID(run1414068597716) ENABLED START*/  // Add additional options here
    callCounter++;
    cout << "ConstraintUsingBehaviour was called " << callCounter << " times!" << endl;
    query->getSolution(SolverType::DUMMYSOLVER, runningPlan, result);
    /*PROTECTED REGION END*/
}
void ConstraintUsingBehaviour::initialiseParameters() {
    /*PROTECTED REGION ID(initialiseParameters1414068597716) ENABLED START*/  // Add additional options here
    this->query = make_shared<alica::Query>();
    query->addStaticVariable(getVariableByName("Y"));
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1414068597716) ENABLED START*/  // Add additional methods here
int ConstraintUsingBehaviour::getCallCounter() {
    return callCounter;
}
/*PROTECTED REGION END*/
} /* namespace alica */
