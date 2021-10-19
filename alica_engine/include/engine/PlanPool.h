#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <typeindex>

namespace alica
{
class AlicaEngine;
class Plan;
class Configuration;
class BasicPlan;
class ConfAbstractPlanWrapper;
class IPlanCreator;
class RunningPlan;

/**
 * Manages the connection between the domain specific implementation (BasicPlan) of Plans.
 * It creates used BasicPlan with its given PlanCreator and starts and stops the Plans.
 */
class PlanPool
{
public:
    PlanPool(AlicaEngine* ae);
    ~PlanPool();
    bool init(IPlanCreator& bc);
    void startPlan(RunningPlan& rp);
    void stopPlan(RunningPlan& rp);
    const std::map<const ConfAbstractPlanWrapper*, std::unique_ptr<BasicPlan>>& getAvailablePlans() const { return _availablePlans; }

private:
    BasicPlan* getBasicPlan(const Plan* plan, const Configuration* configuration) const;
    /**
     * Manages plans used by the running ALICA program.
     * The key of the map is the ConfAbstractPlanWrapper, which is implicitly created through the PlanDesigner.
     * The value is the basic plan, which is the implementation of that plan.
     */
    std::map<const ConfAbstractPlanWrapper*, std::unique_ptr<BasicPlan>> _availablePlans;
    AlicaEngine* _ae;
};

} /* namespace alica */
