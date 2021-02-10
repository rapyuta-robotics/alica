#pragma once

#include <map>
#include <memory>

namespace alica
{
class AlicaEngine;
class Plan;
class BasicPlan;
class ConfAbstractPlanWrapper;
class IPlanCreator;
class RunningPlan;
class Configuration;

class PlanPool {
public:
    PlanPool(AlicaEngine* ae);
    ~PlanPool();
    bool init(IPlanCreator& planCreator);
    void setPlan(RunningPlan& rp);
    const std::shared_ptr<BasicPlan> getBasicPlan(const Plan* plan, const Configuration* configuration) const;

private:
    std::map<const ConfAbstractPlanWrapper*, std::shared_ptr<BasicPlan>> _availablePlans;
    AlicaEngine* _ae;
};

}
