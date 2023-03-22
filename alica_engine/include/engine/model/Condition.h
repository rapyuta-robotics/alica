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
class ConditionFactory;
class ExpressionHandler;
class Blackboard;

/**
 * A condition encapsulates expressions and constraint specific to a AlicaElement, e.g., a Transition, or a Plan.
 */
class Condition : public AlicaElement
{
public:
    Condition();

    /**
     * The delegate type used to attach constraints to plans.
     */
    void getConstraint(std::shared_ptr<ProblemDescriptor> pd, const RunningPlan& rp) const;

    void setAbstractPlan(const AbstractPlan* abstractPlan);
    const AbstractPlan* getAbstractPlan() const { return _abstractPlan; }

    void setConditionString(const std::string& conditionString);
    const std::string& getConditionString() const { return _conditionString; }

    void addVariable(const Variable* variables);
    const VariableGrp& getVariables() const { return _variables; }
    void addQuantifier(const Quantifier* quantifier);
    const QuantifierGrp& getQuantifiers() const { return _quantifiers; }

    const std::shared_ptr<BasicCondition>& getBasicCondition() const { return _basicCondition; }

    bool evaluate(const RunningPlan& rp, const Blackboard* globalBlackboard) const;

    void setLibraryName(const std::string& name);
    std::string getLibraryName() const { return _libraryName; }

    void setBasicConstraint(const std::shared_ptr<BasicConstraint>& basicConstraint);
    void setBasicCondition(const std::shared_ptr<BasicCondition>& basicCondition);

private:
    std::shared_ptr<BasicCondition> _basicCondition;
    std::shared_ptr<BasicConstraint> _basicConstraint;

    /**
     * The variables used in the constraint of this condition.
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
    std::string _libraryName;
};
} // namespace alica
