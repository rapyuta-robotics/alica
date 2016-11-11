using namespace std;
#include "Plans/GSolver/SolverTestBehaviour.h"

/*PROTECTED REGION ID(inccpp1417424455986) ENABLED START*/ //Add additional includes here
#include "SolverType.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417424455986) ENABLED START*/ //initialise static variables here
    vector<double> SolverTestBehaviour::result;
    /*PROTECTED REGION END*/
    SolverTestBehaviour::SolverTestBehaviour() :
            DomainBehaviour("SolverTestBehaviour")
    {
        /*PROTECTED REGION ID(con1417424455986) ENABLED START*/ //Add additional options here
        this->callCounter = 0;

        /*PROTECTED REGION END*/
    }
    SolverTestBehaviour::~SolverTestBehaviour()
    {
        /*PROTECTED REGION ID(dcon1417424455986) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void SolverTestBehaviour::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417424455986) ENABLED START*/ //Add additional options here
        callCounter++;
        //cout << "SolverTestBehaviour was called " << callCounter << " times!" << endl;
        query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
        /*PROTECTED REGION END*/
    }
    void SolverTestBehaviour::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417424455986) ENABLED START*/ //Add additional options here
    	this->query = make_shared < alica::Query > (this->getRunningPlan()->getAlicaEngine());
        query->clearStaticVariables();
        query->addVariable(getVariablesByName("X"));
        query->addVariable(getVariablesByName("Y"));
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417424455986) ENABLED START*/ //Add additional methods here
    int SolverTestBehaviour::getCallCounter()
    {
        return callCounter;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
