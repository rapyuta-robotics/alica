#pragma once

#include "engine/RunnableObject.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

protected:
    virtual void onInit(){};
    virtual void run(void* msg){};
    virtual void onTerminate(){};
    virtual const std::unordered_map<int64_t, std::unordered_set<int64_t>>& getApplicationEntrypointContext(std::vector<int64_t> entryPointIds);

private:
    void doInit() override;
    void doRun(void* msg);
    void doTerminate() override;
};
} // namespace alica
