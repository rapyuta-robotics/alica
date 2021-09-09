#pragma once

namespace alica
{
class RunningPlan;
class Plan;
class Condition;
class PlanRepository;
class AlicaCreators;
class IConstraintCreator;
class AlicaEngine;

/**
 * The ExpressionHandler attaches expressions and constraints to plans during start-up of the engine.
 */
class ExpressionHandler
{
public:
    ExpressionHandler();
    virtual ~ExpressionHandler();
    void attachAll(AlicaEngine* ae, PlanRepository& pr, AlicaCreators& creatorCtx);

private:
    void attachConstraint(Condition* c, IConstraintCreator& crc);
};

} // namespace alica
