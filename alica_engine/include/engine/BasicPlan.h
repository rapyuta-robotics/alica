#pragma once

#include "engine/RunnableObject.h"

namespace alica
{

class BasicPlan : private RunnableObject
{
public:
    BasicPlan();
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

protected:
    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun(void* msg);
    void doTerminate() override;
};
} // namespace alica
