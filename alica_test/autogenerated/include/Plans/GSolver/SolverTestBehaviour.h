#ifndef SolverTestBehaviour_H_
#define SolverTestBehaviour_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417424455986) ENABLED START*/ //Add additional includes here
#include <vector>
#include <engine/constraintmodul/Query.h>
/*PROTECTED REGION END*/
namespace alica
{
    class SolverTestBehaviour : public DomainBehaviour
    {
    public:
        SolverTestBehaviour();
        virtual ~SolverTestBehaviour();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417424455986) ENABLED START*/ //Add additional public methods here
        int getCallCounter();
        static vector<double> result;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417424455986) ENABLED START*/ //Add additional protected methods here
        alica::Query _query;
        int callCounter;

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417424455986) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* SolverTestBehaviour_H_ */
