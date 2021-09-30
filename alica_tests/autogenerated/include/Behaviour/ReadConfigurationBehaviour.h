#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1588061129360) ENABLED START*/
/*PROTECTED REGION END*/

namespace alica
{
class ReadConfigurationBehaviour : public DomainBehaviour
{
public:
    ReadConfigurationBehaviour();
    virtual ~ReadConfigurationBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1588061129360) ENABLED START*/
    std::string testValue;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1588061129360) ENABLED START*/
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1588061129360) ENABLED START*/
    /*PROTECTED REGION END*/
};
} /* namespace alica */
