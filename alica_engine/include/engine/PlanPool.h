#pragma once

namespace alica
{
class AlicaEngine;
class Plan;
class BasicPlan;
class ConfAbstractPlanWrapper;
class IPlanCreator;
class RunningPlan;

class PlanPool {
public:
    PlanPool(AlicaEngine* ae);
    ~PlanPool();
    bool init(IPlanCreator& planCreator);
    void setPlan(RunningPlan& rp);

private:
    std::map<const ConfAbstractPlanWrapper*, std::shared_ptr<BasicPlan>> _availablePlans;
    AlicaEngine* _ae;
};

}
