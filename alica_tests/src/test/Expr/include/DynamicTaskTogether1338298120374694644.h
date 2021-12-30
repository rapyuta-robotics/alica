#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1338298120374694644) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1338298120374694644) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class DynamicTaskTogether1338298120374694644 : public DomainPlan
{
public:
    DynamicTaskTogether1338298120374694644(IAlicaWorldModel* wm);
    virtual ~DynamicTaskTogether1338298120374694644();
    /*PROTECTED REGION ID(pub1338298120374694644) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual bool getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap) override;
    /*PROTECTED REGION ID(pro1338298120374694644) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1338298120374694644) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1338298120374694644 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
