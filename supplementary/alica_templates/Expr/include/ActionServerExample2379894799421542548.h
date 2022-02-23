#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl2379894799421542548) ENABLED START*/
// Add additional includes here
#include <actionlib/server/simple_action_server.h>
#include <alica_templates/DummyAction.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2379894799421542548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class ActionServerExample2379894799421542548 : public DomainPlan
{
public:
    ActionServerExample2379894799421542548(IAlicaWorldModel* wm);
    virtual ~ActionServerExample2379894799421542548();
    /*PROTECTED REGION ID(pub2379894799421542548) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro2379894799421542548) ENABLED START*/
    // Override these methods for your use case
    virtual void run(void* msg) override;
    virtual void onInit() override;
    virtual void onTerminate() override;
    void goalCallback();
    void preemptCallback();
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2379894799421542548) ENABLED START*/
    // Add additional private methods here
    ros::NodeHandle _nh;
    std::unique_ptr<actionlib::SimpleActionServer<alica_templates::DummyAction>> _actionServer;
    alica_templates::DummyFeedback _feedback;
    alica_templates::DummyResult _result;
    /*PROTECTED REGION END*/
};

class UtilityFunction2379894799421542548 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1886820548377048134 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition587249152722263568 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
