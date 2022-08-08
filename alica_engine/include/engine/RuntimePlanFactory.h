#pragma once

#include <iostream>
#include <memory>

namespace alica
{

class AlicaEngine;
class Plan;
class BasicPlan;
class IPlanCreator;
class IAlicaWorldModel;
class IAlicaLogger;

/**
 * Construct a runtime BasicPlan instance based
 */
class RuntimePlanFactory
{
public:
    // TODO: remove engine reference later
    RuntimePlanFactory(std::unique_ptr<IPlanCreator>&& pc, IAlicaWorldModel* wm, AlicaEngine* engine);
    ~RuntimePlanFactory() = default;

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

private:
    std::unique_ptr<IPlanCreator> _creator;
    IAlicaWorldModel* _wm;
    AlicaEngine* _engine;
};

} /* namespace alica */
