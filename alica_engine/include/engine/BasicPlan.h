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
    using RunnableObject::getWorldModel;
    using RunnableObject::getName;

    void stop();
    void start(RunningPlan* rp);
    void setAsMasterPlan() { _isMasterPlan = true; };

protected:
    // Set the tracing type for this plan. customTraceContextGetter is required for custom tracing
    // & this method will be called to get the parent trace context before onInit is called
    void setTracing(TracingType type, std::function<std::optional<std::string>(BasicPlan*)> customTraceContextGetter = {})
    {
        _tracingType = type;
        _customTraceContextGetter = [this, customTraceContextGetter]() {
            return customTraceContextGetter(this);
        };
    }

    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void initJob();
    void runJob();
    void terminateJob();

    void onInit_() override;
    void onRun_() override;
    void onTerminate_() override;

    bool _isMasterPlan;
};
} // namespace alica
