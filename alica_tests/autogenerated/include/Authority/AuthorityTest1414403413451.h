#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1414403413451) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1414403413451) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class AuthorityTest1414403413451 : public DomainPlan
{
public:
    AuthorityTest1414403413451();
    virtual ~AuthorityTest1414403413451();
    /*PROTECTED REGION ID(pub1414403413451) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro1414403413451) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1414403413451) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1414403413451 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
