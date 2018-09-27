using namespace std;
#include "Plans/Behaviour/ConstraintUsingBehaviour.h"

/*PROTECTED REGION ID(inccpp1414068597716) ENABLED START*/ // Add additional includes here
#include "ConstraintTestPlanDummySolver.h"
#include <iostream>

/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1414068597716) ENABLED START*/ // initialise static variables here
std::vector<BBIdent> ConstraintUsingBehaviour::result;
/*PROTECTED REGION END*/
ConstraintUsingBehaviour::ConstraintUsingBehaviour()
    : DomainBehaviour("ConstraintUsingBehaviour")
{
    /*PROTECTED REGION ID(con1414068597716) ENABLED START*/ // Add additional options here
    _callCounter = 0;
    /*PROTECTED REGION END*/
}
ConstraintUsingBehaviour::~ConstraintUsingBehaviour()
{
    /*PROTECTED REGION ID(dcon1414068597716) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void ConstraintUsingBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run1414068597716) ENABLED START*/ // Add additional options here
    ++_callCounter;
    cout << "ConstraintUsingBehaviour was called " << _callCounter << " times!" << endl;
    _query.getSolution<reasoner::ConstraintTestPlanDummySolver, BBIdent>(runningPlan, result);
    /*PROTECTED REGION END*/
}
void ConstraintUsingBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1414068597716) ENABLED START*/ // Add additional options here
    _callCounter = 0;
    _query.clearStaticVariables();
    _query.addStaticVariable(getVariableByName("Y"));
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1414068597716) ENABLED START*/ // Add additional methods here
int ConstraintUsingBehaviour::getCallCounter() const
{
    return _callCounter;
}
/*PROTECTED REGION END*/
} /* namespace alica */
