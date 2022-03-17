#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl725594143882346503) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth725594143882346503) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PickDrop725594143882346503 : public DomainPlan
{
public:
    PickDrop725594143882346503(IAlicaWorldModel* wm);
    virtual ~PickDrop725594143882346503();
    /*PROTECTED REGION ID(pub725594143882346503) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual bool getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap) override;
    /*PROTECTED REGION ID(pro725594143882346503) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv725594143882346503) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class PlanAttachment1463506596775213702 : public PlanAttachment
{
    bool setParameters(const Blackboard& parent_bb, Blackboard& child_bb);
};
class PlanAttachment2743493125610368794 : public PlanAttachment
{
    bool setParameters(const Blackboard& parent_bb, Blackboard& child_bb);
};
class UtilityFunction725594143882346503 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition3953991713597643491 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition32970225314063392 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition232513088105009661 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition3691801807787093963 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
