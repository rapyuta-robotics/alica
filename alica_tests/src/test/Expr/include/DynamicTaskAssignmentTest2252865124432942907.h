#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl2252865124432942907) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2252865124432942907) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class DynamicTaskAssignmentTest2252865124432942907 : public DomainPlan
{
public:
    DynamicTaskAssignmentTest2252865124432942907(IAlicaWorldModel* wm);
    virtual ~DynamicTaskAssignmentTest2252865124432942907();
    /*PROTECTED REGION ID(pub2252865124432942907) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual bool getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap) override;
    /*PROTECTED REGION ID(pro2252865124432942907) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2252865124432942907) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction2252865124432942907 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1078898265232036813 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
