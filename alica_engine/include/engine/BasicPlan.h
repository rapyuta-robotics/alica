#pragma once

#include "engine/IPlanCreator.h"
#include "engine/RunnableObjectNew.h"
#include "engine/blackboard/KeyMapping.h"

#include <unordered_map>

namespace alica
{

class Plan;

struct PlanContext
{
    IAlicaWorldModel* worldModel;
    const std::string name;
    const Plan* planModel;
};

class BasicPlan : private RunnableObjectNew
{
public:
    BasicPlan(PlanContext& context);
    virtual ~BasicPlan() = default;

    // Use of private inheritance and explicitly making members public
    // to share code between BasicPlan and Runnable object but not expose internals to further derived classes
    using RunnableObjectNew::getBlackboard;
    using RunnableObjectNew::getInheritBlackboard;
    using RunnableObjectNew::getKeyMapping;
    using RunnableObjectNew::getName;
    using RunnableObjectNew::getPlanContext;
    using RunnableObjectNew::getTrace;
    using RunnableObjectNew::getWorldModel;
    using RunnableObjectNew::setEngine;
    using RunnableObjectNew::start;
    using RunnableObjectNew::stop;
    using RunnableObjectNew::TracingType;

    void traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);
    int64_t getId() const;

protected:
    void setTracing(TracingType type, std::function<std::optional<std::string>(const BasicPlan*)> customTraceContextGetter = {})
    {
        if (customTraceContextGetter) {
            RunnableObjectNew::setTracing(
                    type, [this, customTraceContextGetter = std::move(customTraceContextGetter)]() { return customTraceContextGetter(this); });
        } else {
            RunnableObjectNew::setTracing(type, {});
        }
    }

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun() override;
    void doTerminate() override;

    bool _isMasterPlan;
    const Plan* _plan;
};
} // namespace alica
