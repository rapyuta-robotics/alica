#pragma once

#include "engine/IPlanCreator.h"
#include "engine/RunnableObject.h"
#include "engine/Types.h"
#include "engine/blackboard/KeyMapping.h"

#include <memory>
#include <unordered_map>

namespace alica
{

class Plan;
class Transition;

struct PlanContext
{
    std::shared_ptr<Blackboard> globalBlackboard;
    const std::string name;
    const Plan* planModel;
    const IAlicaTraceFactory* traceFactory;
};

class BasicPlan : private RunnableObject
{
public:
    BasicPlan(PlanContext& context);
    virtual ~BasicPlan() = default;

    // Use of private inheritance and explicitly making members public
    // to share code between BasicPlan and Runnable object but not expose internals to further derived classes
    using RunnableObject::getBlackboard;
    using RunnableObject::getBlackboardBlueprint;
    using RunnableObject::getGlobalBlackboard;
    using RunnableObject::getInheritBlackboard;
    using RunnableObject::getKeyMapping;
    using RunnableObject::getName;
    using RunnableObject::getPlanContext;
    using RunnableObject::getTrace;
    using RunnableObject::setAlicaTimerFactory;
    using RunnableObject::setAlicaTraceFactory;
    using RunnableObject::setTeamManager;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::TracingType;

    void traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);
    int64_t getId() const;

    static std::unique_ptr<BasicPlan> create(PlanContext& context);

protected:
    using RunnableObject::getTraceFactory;

    void setTracing(TracingType type, std::function<tracing::SpanStartOptions(const BasicPlan*)> customTraceContextGetter = {})
    {
        if (customTraceContextGetter) {
            RunnableObject::setTracing(
                    type, [this, customTraceContextGetter = std::move(customTraceContextGetter)]() { return customTraceContextGetter(this); });
        } else {
            RunnableObject::setTracing(type, {});
        }
    }
    void setTracing(TracingType type, std::function<std::optional<std::string>(const BasicPlan*)> customTraceContextGetter)
    {
        setTracing(type, [this, inner = std::move(customTraceContextGetter)](const BasicPlan*) -> tracing::SpanStartOptions {
            tracing::SpanStartOptions options;
            options.parentContext = inner(this);
            return options;
        });
    }

    virtual void onInit(){};
    virtual void run(){};
    virtual void onTerminate(){};

private:
    void doInit() override;
    void doRun() override;
    void doTerminate() override;

    const Plan* _plan;
};
} // namespace alica
