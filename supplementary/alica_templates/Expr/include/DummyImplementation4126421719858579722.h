#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl4126421719858579722) ENABLED START*/
// Add additional includes here
#include <alica_msgs/DummyAction.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth4126421719858579722) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class DummyImplementation4126421719858579722 : public DomainPlan
{
public:
    DummyImplementation4126421719858579722(IAlicaWorldModel* wm);
    virtual ~DummyImplementation4126421719858579722();
    /*PROTECTED REGION ID(pub4126421719858579722) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro4126421719858579722) ENABLED START*/
    // Override these methods for your use case
    virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4126421719858579722) ENABLED START*/
    // Add additional private methods here
    int32_t _currentGoal;
    /*PROTECTED REGION END*/
};

class UtilityFunction4126421719858579722 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition3469760593538210700 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition2084505765197602547 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
