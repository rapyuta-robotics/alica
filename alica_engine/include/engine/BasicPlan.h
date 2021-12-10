#pragma once

#include "engine/RunnableObject.h"

namespace alica
{

class BasicPlan : private RunnableObject
{
public:
    BasicPlan(IAlicaWorldModel* wm);
    virtual ~BasicPlan() = default;

    // Use of private inheritance and explciltly making members public
    // to share code between BasicPlan and Runnable object but not expose internals to further derived classes
    using RunnableObject::getPlanContext;
    using RunnableObject::getTraceContext;
    using RunnableObject::setConfiguration;
    using RunnableObject::setEngine;
    using RunnableObject::setInterval;
    using RunnableObject::setName;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::getWorldModel;
    using RunnableObject::getName;

    void notifyAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);
    void setAsMasterPlan() { _isMasterPlan = true; };

protected:
    void setTracing(TracingType type, std::function<std::optional<std::string>(const BasicPlan*)> customTraceContextGetter = {})
    {
        if (customTraceContextGetter) {
            RunnableObject::setTracing(type, [this, customTraceContextGetter]() { return customTraceContextGetter(this); });
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
};
} // namespace alica
