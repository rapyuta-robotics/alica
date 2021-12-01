#pragma once

#include "engine/RunnableObject.h"
#include "engine/PlanAttachment.h"
#include "engine/IPlanCreator.h"

#include <unordered_map>
namespace alica
{

class Plan;

class BasicPlan : private RunnableObject
{
public:
    BasicPlan(IAlicaWorldModel* wm);
    virtual ~BasicPlan() = default;

    // Use of private inheritance and explicitly making members public
    // to share code between BasicPlan and Runnable object but not expose internals to further derived classes
    using RunnableObject::getPlanContext;
    using RunnableObject::getTraceContext;
    using RunnableObject::setConfiguration;
    using RunnableObject::setEngine;
    using RunnableObject::setRequiresParameters;
    using RunnableObject::setInterval;
    using RunnableObject::setName;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::getBlackBoard;
    using RunnableObject::getWorldModel;

    void createChildAttachments(const Plan* plan, IPlanCreator& planCreator);

    std::unique_ptr<PlanAttachment>& getPlanAttachment(int64_t id) {return _planAttachments.at(id);}
protected:
    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun(void* msg);
    void doTerminate() override;

    // Map from ConfAbstractPlanWrapper id to associated attachment
    std::unordered_map<int64_t, std::unique_ptr<PlanAttachment>> _planAttachments;
};
} // namespace alica
