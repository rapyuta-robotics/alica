/*
 * Condition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Condition.h"

#include "engine/model/Quantifier.h"
#include "engine/BasicCondition.h"

namespace alica
{

	Condition::Condition()
	{
		this->abstractPlan = nullptr;
		this->basicCondition = nullptr;
	}

	Condition::Condition(long id)
	{
		this->id = id;
		this->abstractPlan = nullptr;
		this->basicCondition = nullptr;
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

	bool Condition::evaluate(shared_ptr<RunningPlan> rp)
	{
		if (basicCondition == nullptr)
		{
			cerr << "Condition: Missing implementation of Condition: " << this->getId() << endl;
			return false;
		}
		else
		{
			return basicCondition->evaluate(rp);
		}
	}

	void Condition::setQuantifiers(const list<Quantifier*>& quantifiers)
	{
		this->quantifiers = quantifiers;
	}

	vector<Variable*>& Condition::getVariables()
	{
		return variables;
	}

	void Condition::setVariables(const vector<Variable*>& variables)
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

	shared_ptr<BasicCondition> Condition::getBasicCondition()
	{
		return basicCondition;
	}

	void Condition::setBasicCondition(shared_ptr<BasicCondition> basicCondition)
	{
		this->basicCondition = basicCondition;
	}

} /* namespace Alica */


