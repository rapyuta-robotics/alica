#pragma once


#include "AlicaElement.h"
#include "engine/Types.h"

#include <memory>
#include <string>


namespace alica
{
class AbstractPlan;
class BasicCondition;
class BasicConstraint;
class RunningPlan;
class ProblemDescriptor;
class ModelFactory;
class ConditionFactory;
class ExpressionHandler;
class IAlicaWorldModel;

/**
 * A condition encapsulates expressions and constraint specific to a AlicaElement, e.g., a Transition, or a Plan.
 */
class Condition : public AlicaElement
{
public:
    Condition();
    virtual ~Condition();

    /**
     * The delegate type used to attach constraints to plans.
     */
    void getConstraint(std::shared_ptr<ProblemDescriptor> pd, const RunningPlan& rp) const;

    const AbstractPlan* getAbstractPlan() const { return _abstractPlan; }

    const std::string& getConditionString() const { return _conditionString; }
    const std::string& getPlugInName() const { return _plugInName; }

    const VariableGrp& getVariables() const { return _variables; }
    const QuantifierGrp& getQuantifiers() const { return _quantifiers; }

    const std::shared_ptr<BasicCondition>& getBasicCondition() const { return _basicCondition; }

    bool evaluate(const RunningPlan& rp, const IAlicaWorldModel* wm) const;

private:
    friend ModelFactory;
    friend ConditionFactory;
    friend ExpressionHandler;

    void setConditionString(const std::string& conditionString);
    void setVariables(const VariableGrp& variables);
    void setPlugInName(const std::string& plugInName);
    void setAbstractPlan(const AbstractPlan* abstractPlan);
    void setBasicConstraint(const std::shared_ptr<BasicConstraint>& basicConstraint);
    void setBasicCondition(const std::shared_ptr<BasicCondition>& basicCondition);
    void setQuantifiers(const QuantifierGrp& quantifiers);

    std::shared_ptr<BasicCondition> _basicCondition;
    std::shared_ptr<BasicConstraint> _basicConstraint;

    /**
     * The static variables used in the constraint of this condition.
     */
    VariableGrp _variables;

    /**
     * The quantifiers used in the constraint of this condition.
     */
    QuantifierGrp _quantifiers;

    /**
     * The Abstract Plan in which this condition occurs.
     */
    const AbstractPlan* _abstractPlan;

    std::string _conditionString;
    std::string _plugInName; // TODO: is this needed?!
};
} // namespace alica
