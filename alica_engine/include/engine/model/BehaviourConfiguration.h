#pragma once
#include "AbstractPlan.h"
#include "engine/Types.h"

 namespace alica {
 class Behaviour;
 class ModelFactory;
 class BehaviourConfigurationFactory;

/**
 * A Behaviour Configuration encapsulates a set of static parameters for a (Basic)Behaviour.
 *
 * The BehaviourConfiguration is indirectly derived from AlicaElement, therefore it
 * is owned by the PlanRepository and should never be deleted or changed by anybody else.
 */
 class BehaviourConfiguration : public AlicaElement {
 public:
    BehaviourConfiguration();
    virtual ~BehaviourConfiguration();

     std::string toString(std::string indent = "") const override;

    const BehaviourParameterMap& getParameters() const { return _parameters; }
    const Behaviour* getBehaviour() const { return _behaviour; }

 private:
     friend BehaviourConfigurationFactory;
    void setParameters(const BehaviourParameterMap& parameters);
    void setBehaviour(const Behaviour* behaviour);
    /**
     * The set of static parameters of this Behaviour configuration. Usually parsed by
     * BasicBehaviour.InitializeParameters.
     */
    BehaviourParameterMap _parameters;
    /**
     * This configuration's Behaviour
     */
    const Behaviour* _behaviour;
};

}  // namespace alica
