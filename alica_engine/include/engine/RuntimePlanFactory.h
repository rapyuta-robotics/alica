#pragma once

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace alica
{

class AlicaEngine;
class Plan;
class BasicPlan;
class IPlanCreator;
class IAlicaWorldModel;
class ConfigChangeListener;

/**
 * Construct a runtime BasicPlan instance based
 */
class RuntimePlanFactory
{
public:
    // TODO: remove engine reference later
    RuntimePlanFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, AlicaEngine* engine);
    ~RuntimePlanFactory() = default;
    void init(std::unique_ptr<IPlanCreator>&& pc);

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

    void reload(const YAML::Node& config);

private:
    std::unique_ptr<IPlanCreator> _creator;
    IAlicaWorldModel* _wm;
    AlicaEngine* _engine;

    std::string _customerLibraryFolder;
};

} /* namespace alica */
