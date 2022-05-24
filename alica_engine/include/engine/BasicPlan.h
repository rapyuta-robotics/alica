#pragma once

#include "engine/IPlanCreator.h"
#include "engine/RunnableObject.h"
#include "engine/BasicTransitionCondition.h"
#include "engine/Types.h"
#include "engine/blackboard/KeyMapping.h"

#include <unordered_map>
#include <memory>

namespace alica
{

class Plan;
class Transition;

struct PlanContext
{
    IAlicaWorldModel* worldModel;
    const std::string name;
    const Plan* planModel;
};

class BasicPlan : private RunnableObject
{
public:
    BasicPlan(PlanContext& context);
    virtual ~BasicPlan() = default;

    // Use of private inheritance and explicitly making members public
    // to share code between BasicPlan and Runnable object but not expose internals to further derived classes
    using RunnableObject::getBlackboard;
    using RunnableObject::getInheritBlackboard;
    using RunnableObject::getKeyMapping;
    using RunnableObject::getName;
    using RunnableObject::getPlanContext;
    using RunnableObject::getTrace;
    using RunnableObject::getWorldModel;
    using RunnableObject::setEngine;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::TracingType;

    void traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents);
    int64_t getId() const;
    bool evalTransitionCondition(const Transition* transition, const RunningPlan* rp, const IAlicaWorldModel* wm);
    void initTransitionConditions(const TransitionGrp& transitions);

protected:
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
    void doRun() override;
    void doTerminate() override;

    bool _isMasterPlan;
    const Plan* _plan;
    std::unordered_map<int64_t, std::unique_ptr<BasicTransitionCondition>> _transitionConditions;
};
} // namespace alica
