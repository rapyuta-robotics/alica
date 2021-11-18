#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc1479556104511) ENABLED START*/
// Add additional includes here
#include <engine/constraintmodul/Query.h>
#include <mutex>

/*PROTECTED REGION END*/

namespace alica
{
class QueryBehaviour1 : public DomainBehaviour
{
public:
    QueryBehaviour1(IAlicaWorldModel* wm);
    virtual ~QueryBehaviour1();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1479556104511) ENABLED START*/
    // Add additional public methods here
    int getCallCounter();
    void stopQueries();

    static std::vector<double> result;
    std::shared_ptr<alica::Query> query;
    std::mutex queryMutex;
    bool stopQuerying;

    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1479556104511) ENABLED START*/
    // Add additional protected methods here
    int callCounter;
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1479556104511) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
