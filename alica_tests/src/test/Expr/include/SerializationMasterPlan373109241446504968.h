#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl373109241446504968) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth373109241446504968) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SerializationMasterPlan373109241446504968 : public DomainPlan
{
public:
    SerializationMasterPlan373109241446504968(IAlicaWorldModel* wm);
    virtual ~SerializationMasterPlan373109241446504968();
    /*PROTECTED REGION ID(pub373109241446504968) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro373109241446504968) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv373109241446504968) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction373109241446504968 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1693256954385338259 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition3932287302905544988 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition3461968191733792853 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition2915681556800498724 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
