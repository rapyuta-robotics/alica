#ifndef QueryBehaviour1_H_
#define QueryBehaviour1_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1479556104511) ENABLED START*/ // Add additional includes here
#include <engine/constraintmodul/Query.h>
/*PROTECTED REGION END*/
namespace alica
{
class QueryBehaviour1 : public DomainBehaviour
{
public:
    QueryBehaviour1();
    virtual ~QueryBehaviour1();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1479556104511) ENABLED START*/ // Add additional public methods here
    int getCallCounter();
    static vector<double> result;
    shared_ptr<alica::Query> query;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1479556104511) ENABLED START*/ // Add additional protected methods here
    int callCounter;
    /*PROTECTED REGION END*/
private:
/*PROTECTED REGION ID(prv1479556104511) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* QueryBehaviour1_H_ */
