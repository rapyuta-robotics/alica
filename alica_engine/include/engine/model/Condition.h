/*
 * Condition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CONDITION_H_
#define CONDITION_H_

#include <string>
#include <memory>

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica {
class AbstractPlan;
class BasicCondition;
class BasicConstraint;
class RunningPlan;
class ProblemDescriptor;
class ModelFactory;
class ExpressionHandler;

/**
 * A condition encapsulates expressions and constraint specific to a AlicaElement, e.g., a Transition, or a Plan.
 */
class Condition : public AlicaElement {
public:
    Condition();
    Condition(int64_t id);
    virtual ~Condition();

    /**
     * The delegate type used to attach constraints to plans.
     */
    void getConstraint(std::shared_ptr<ProblemDescriptor> pd, std::shared_ptr<RunningPlan> rp) const;

    const AbstractPlan* getAbstractPlan() const {return _abstractPlan;}

    const std::string& getConditionString() const {return _conditionString;}
    const std::string& getPlugInName() const {return _plugInName;}

    const VariableSet& getVariables() const {return _variables;}
    const ParameterSet& getParameters() const {return _parameters;}
    const QuantifierSet& getQuantifiers() const {return _quantifiers;}

    const std::shared_ptr<BasicCondition>& getBasicCondition() const {return _basicCondition;}

    bool evaluate(std::shared_ptr<RunningPlan> rp) const;

private:
    friend ModelFactory;
    friend ExpressionHandler;

    void setConditionString(const std::string& conditionString);
    void setVariables(const VariableSet& variables);
    void setPlugInName(const std::string& plugInName);
    void setAbstractPlan(const AbstractPlan* abstractPlan);
    void setParameters(const ParameterSet& parameters);
    void setBasicConstraint(const std::shared_ptr<BasicConstraint>& basicConstraint);
    void setBasicCondition(const std::shared_ptr<BasicCondition>& basicCondition);
    void setQuantifiers(const QuantifierSet& quantifiers);

    std::shared_ptr<BasicCondition> _basicCondition;
    std::shared_ptr<BasicConstraint> _basicConstraint;
    ParameterSet _parameters;

    /**
     * The static variables used in the constraint of this condition.
     */
    VariableSet _variables;

    /**
     * The quantifiers used in the constraint of this condition.
     */
    QuantifierSet _quantifiers;

    /**
     * The Abstract Plan in which this condition occurs.
     */
    const AbstractPlan* _abstractPlan;

    std::string _conditionString;
    std::string _plugInName; //TODO: is this needed?!
};
}  // namespace alica

#endif /* CONDITION_H_ */
