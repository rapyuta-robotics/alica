/*
 * Condition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Condition.h"

#include "engine/BasicCondition.h"
#include "engine/BasicConstraint.h"
#include "engine/model/Quantifier.h"

namespace alica
{

Condition::Condition()
    : _abstractPlan(nullptr)
    , _basicCondition(nullptr)
{
}

Condition::Condition(int64_t id)
    : AlicaElement(id)
    , _abstractPlan(nullptr)
    , _basicCondition(nullptr)
{
}

Condition::~Condition() {}

void Condition::getConstraint(std::shared_ptr<ProblemDescriptor> pd, std::shared_ptr<const RunningPlan> rp) const
{
    // TODO: fix const cast below
    _basicConstraint->getConstraint(pd, std::const_pointer_cast<RunningPlan>(rp));
}

void Condition::setConditionString(const std::string& conditionString)
{
    _conditionString = conditionString;
}

bool Condition::evaluate(std::shared_ptr<RunningPlan> rp) const
{
    if (_basicCondition == nullptr) {
        std::cerr << "Condition: Missing implementation of condition: ID " << getId() << std::endl;
        return false;
    } else {
        bool ret = false;
        try {
            ret = _basicCondition->evaluate(rp);
        } catch (std::exception& e) {
            std::cerr << "Condition: Exception during evaluation catched: " << std::endl << e.what() << std::endl;
        }
        return ret;
    }
}

void Condition::setQuantifiers(const QuantifierGrp& quantifiers)
{
    _quantifiers = quantifiers;
}

void Condition::setVariables(const VariableGrp& variables)
{
    _variables = variables;
}

void Condition::setAbstractPlan(const AbstractPlan* abstractPlan)
{
    _abstractPlan = abstractPlan;
}

void Condition::setPlugInName(const std::string& plugInName)
{
    _plugInName = plugInName;
}

void Condition::setBasicCondition(const std::shared_ptr<BasicCondition>& basicCondition)
{
    _basicCondition = basicCondition;
}

void Condition::setParameters(const ParameterGrp& parameters)
{
    _parameters = parameters;
}

void Condition::setBasicConstraint(const std::shared_ptr<BasicConstraint>& basicConstraint)
{
    _basicConstraint = basicConstraint;
}

} // namespace alica
