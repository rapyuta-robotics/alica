#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1530004915640) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1530004915640) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class HandleFailExplicit1530004915640 : public DomainPlan
{
public:
    HandleFailExplicit1530004915640(PlanContext& context);
    virtual ~HandleFailExplicit1530004915640();
    /*PROTECTED REGION ID(pub1530004915640) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1530004915640) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1530004915640) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1530004915640 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1530004993493 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1530004994611 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1532424093178 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1532424113475 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
