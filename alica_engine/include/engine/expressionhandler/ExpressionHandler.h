#pragma once

#include <yaml-cpp/yaml.h>

namespace alica
{
class RunningPlan;
class Plan;
class Condition;
class PlanRepository;
class AlicaCreators;
class IConstraintCreator;
class AlicaEngine;
class ConfigChangeListener;

/**
 * The ExpressionHandler attaches expressions and constraints to plans during start-up of the engine.
 */
class ExpressionHandler
{
public:
    ExpressionHandler(ConfigChangeListener& configChangeListener);
    virtual ~ExpressionHandler();
    void attachAll(PlanRepository& pr, AlicaCreators& creatorCtx);

    void reload(const YAML::Node& config);

private:
    void attachConstraint(Condition* c, IConstraintCreator& crc);
};

} // namespace alica
