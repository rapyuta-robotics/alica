#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1613378382024) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1613378382024) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SchedulingTestMasterPlan1613378382024 : public DomainPlan
{
public:
    SchedulingTestMasterPlan1613378382024();
    virtual ~SchedulingTestMasterPlan1613378382024();
    virtual void run(void* msg) override;
    /*PROTECTED REGION ID(pub1613378382024) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1613378382024) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1613378382024) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1613378382024 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1613530643882 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1615797316171 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1615797327077 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1615797365364 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
