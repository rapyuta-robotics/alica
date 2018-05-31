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

#include <alica_common_config/debug_output.h>

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

void Condition::getConstraint(std::shared_ptr<ProblemDescriptor> pd, const RunningPlan& rp) const
{
    // TODO: fix const cast below
    _basicConstraint->getConstraint(pd, const_cast<RunningPlan&>(rp).getSharedPointer());
}

void Condition::setConditionString(const std::string& conditionString)
{
    _conditionString = conditionString;
}

bool Condition::evaluate(const RunningPlan& rp) const
{
    if (_basicCondition == nullptr) {
        ALICA_ERROR_MSG("Condition: Missing implementation of condition: ID " << getId());
        return false;
    } else {
        bool ret = false;
        try {
            // TODO: fix this:

            ret = _basicCondition->evaluate(const_cast<RunningPlan&>(rp).getSharedPointer());
        } catch (std::exception& e) {
            ALICA_ERROR_MSG("Condition: Exception during evaluation catched: " << std::endl << e.what());
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
