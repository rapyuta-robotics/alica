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
    void stopAll();
    const std::map<std::pair<const Plan*, const Configuration*>, std::unique_ptr<BasicPlan>>& getAvailablePlans() const { return _availablePlans; }
    BasicPlan* getBasicPlan(const Plan* plan, const Configuration* configuration) const;

private:
    std::unique_ptr<BasicPlan> createBasicPlan(IPlanCreator& planCreator, const Plan* plan, const Configuration* configuration);
    /**
     * Manages plans used by the running ALICA program.
     * The key of the map is the ConfAbstractPlanWrapper, which is implicitly created through the PlanDesigner.
     * The value is the basic plan, which is the implementation of that plan.
     */
    std::map<std::pair<const Plan*, const Configuration*>, std::unique_ptr<BasicPlan>> _availablePlans;
    std::unique_ptr<BasicPlan> _masterPlan;
    AlicaEngine* _ae;
};

} /* namespace alica */
