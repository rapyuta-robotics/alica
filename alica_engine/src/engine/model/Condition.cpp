/*
 * Condition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Condition.h"

#include "engine/model/Quantifier.h"
#include "engine/BasicCondition.h"
#include "engine/BasicConstraint.h"

namespace alica
{

	Condition::Condition()
	{
		this->abstractPlan = nullptr;
		this->basicCondition = nullptr;
		this->variables = vector<Variable*>();
		this->quantifiers = list<Quantifier*>();
	}

	Condition::Condition(long id)
	{
		this->id = id;
		this->abstractPlan = nullptr;
		this->basicCondition = nullptr;
		this->variables = vector<Variable*>();
		this->quantifiers = list<Quantifier*>();
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
			std::cerr << "Condition: Missing implementation of condition: ID " << this->getId() << std::endl;
			return false;
		}
		else
		{
			bool ret = false;
			try {
				ret = basicCondition->evaluate(rp);
			} catch (std::exception& e) {
				std::cerr << "Condition: Exception during evaluation catched: " << std::endl << e.what() << std::endl;
			}
			return ret;
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

	list<Parameter*>& Condition::getParameters()
	{
		return parameters;
	}

	void Condition::setParameters(list<Parameter*> parameters)
	{
		this->parameters = parameters;
	}

	void Condition::getConstraint(shared_ptr<ProblemDescriptor> pd, shared_ptr<RunningPlan> rp)
	{
		this->basicConstraint->getConstraint(pd, rp);
	}

	void Condition::setBasicConstraint(shared_ptr<BasicConstraint> basicConstraint)
	{
		this->basicConstraint = basicConstraint;
	}

} /* namespace Alica */


