#pragma once

#include "engine/IPlanCreator.h"
#include "engine/PlanAttachment.h"
#include "engine/RunnableObject.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
    using RunnableObject::getBlackboard;
    using RunnableObject::getName;
    using RunnableObject::getPlanContext;
    using RunnableObject::getRequiresParameters;
    using RunnableObject::getTraceContext;
    using RunnableObject::getWorldModel;
    using RunnableObject::setConfiguration;
    using RunnableObject::setEngine;
    using RunnableObject::setInterval;
    using RunnableObject::setName;
    using RunnableObject::setRequiresParameters;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::TracingType;

    virtual bool getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap);

    void notifyAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);
    void setAsMasterPlan() { _isMasterPlan = true; };

    void createChildAttachments(const Plan* plan, IPlanCreator& planCreator);

    std::unique_ptr<PlanAttachment>& getPlanAttachment(int64_t id) { return _planAttachments.at(id); }

protected:
    using RunnableObject::getTrace;

    void setTracing(TracingType type, std::function<std::optional<std::string>(const BasicPlan*)> customTraceContextGetter = {})
    {
        if (customTraceContextGetter) {
            RunnableObject::setTracing(
                    type, [this, customTraceContextGetter = std::move(customTraceContextGetter)]() { return customTraceContextGetter(this); });
        } else {
            RunnableObject::setTracing(type, {});
        }
    }

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun(void* msg);
    void doTerminate() override;

    void traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);

    bool _isMasterPlan;
    // Map from ConfAbstractPlanWrapper id to associated attachment
    std::unordered_map<int64_t, std::unique_ptr<PlanAttachment>> _planAttachments;
};
} // namespace alica
