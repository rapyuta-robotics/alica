#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
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
    SchedulingTestMasterPlan1613378382024(IAlicaWorldModel* wm);
    virtual ~SchedulingTestMasterPlan1613378382024();
    /*PROTECTED REGION ID(pub1613378382024) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1613378382024) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
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
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition1615797316171 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition1615797327077 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition1629895598471 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition61978004585920576 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition1615797365364 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition1629895607018 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition3213510506830850443 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition68542020926196536 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition4165333637052704488 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
