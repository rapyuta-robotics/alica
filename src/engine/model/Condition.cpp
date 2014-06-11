/*
 * Condition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Condition.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Quantifier.h"

namespace alica
{

	Condition::Condition()
	{
	}

	Condition::Condition(long id)
	{
		this->id = id;
		this->abstractPlan = NULL;
	}

	Condition::~Condition()
	{
	}

	const string& Condition::getConditionString() const
	{
		return conditionString;
	}

	void Condition::setConditionString(const string& conditionString)
	{
		this->conditionString = conditionString;
	}

	list<Quantifier*>& Condition::getQuantifiers()
	{
		return quantifiers;
	}

	void Condition::setQuantifiers(const list<Quantifier*>& quantifiers)
	{
		this->quantifiers = quantifiers;
	}

	const list<Variable*>& Condition::getVariables() const
	{
		return variables;
	}

	void Condition::setVariables(const list<Variable*>& variables)
	{
		this->variables = variables;
	}

	AbstractPlan* Condition::getAbstractPlan() const
	{
		return abstractPlan;
	}

	void Condition::setAbstractPlan(AbstractPlan* abstractPlan)
	{
		this->abstractPlan = abstractPlan;
	}

} /* namespace Alica */
