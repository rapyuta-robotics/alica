#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl691392966514374878) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth691392966514374878) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class TestTracingMasterPlan691392966514374878 : public DomainPlan
{
public:
    TestTracingMasterPlan691392966514374878();
    virtual ~TestTracingMasterPlan691392966514374878();
    /*PROTECTED REGION ID(pub691392966514374878) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro691392966514374878) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv691392966514374878) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction691392966514374878 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1840401110297459509 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
