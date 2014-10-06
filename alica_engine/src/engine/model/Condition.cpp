/*
 * Condition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Condition.h"

namespace alica
{

	Condition::Condition()
	{
		this->abstractPlan = nullptr;
	}

	Condition::Condition(long id)
	{
		this->id = id;
		this->abstractPlan = nullptr;
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

	list<Variable*>& Condition::getVariables()
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

	const string& Condition::getPlugInName() const
	{
		return plugInName;
	}

	void Condition::setPlugInName(const string& plugInName)
	{
		this->plugInName = plugInName;
	}

} /* namespace Alica */
