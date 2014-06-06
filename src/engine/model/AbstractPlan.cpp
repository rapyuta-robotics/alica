/*
 * AbstractPlan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/AbstractPlan.h"

namespace alica
{

	AbstractPlan::AbstractPlan() :
			AlicaElement()
	{
		this->masterPlan = false;
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->authorithyTimeInterval = (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection","MinimalAuthorityTimeInterval") * 1000000;
	}

	AbstractPlan::~AbstractPlan()
	{
	}

	string AbstractPlan::toString() const
	{
		stringstream ss;
		ss << AlicaElement::toString();
		ss << "IsMasterPlan: " << (isMasterPlan() ? "true":"false") << endl;
		return ss.str();
	}

	bool AbstractPlan::containsVar(const Variable* v)
	{
		const list<Variable*> vars = this->getVariables();
		return find(vars.begin(), vars.end(), v) != vars.end();

	}

	bool AbstractPlan::containsVar(string name)
	{
		const list<Variable*> vars = this->getVariables();
		for (Variable* v : vars)
		{
			if (v->getName() == name)
			{
				return true;
			}
		}
		return false;
	}

//============= Getter & Setter =================

	bool AbstractPlan::isMasterPlan() const
	{
		return masterPlan;
	}

	void AbstractPlan::setMasterPlan(bool masterPlan)
	{
		this->masterPlan = masterPlan;
	}

	unsigned long AbstractPlan::getAuthorithyTimeInterval() const
	{
		return authorithyTimeInterval;
	}

	void AbstractPlan::setAuthorithyTimeInterval(unsigned long authorithyTimeInterval)
	{
		this->authorithyTimeInterval = authorithyTimeInterval;
	}

	const string& AbstractPlan::getFileName() const
	{
		return fileName;
	}

	void AbstractPlan::setFileName(const string& fileName)
	{
		this->fileName = fileName;
	}

	const list<Variable*>& AbstractPlan::getVariables() const
	{
		return variables;
	}


	void AbstractPlan::setVariables(const list<Variable*>& variables)
	{
		this->variables = variables;
	}

	const RuntimeCondition* AbstractPlan::getRuntimeCondition() const
	{
		return runtimeCondition;
	}

	void AbstractPlan::setRuntimeCondition(const RuntimeCondition* runtimeCondition)
	{
		this->runtimeCondition = runtimeCondition;
	}

	const PreCondition* AbstractPlan::getPreCondition() const
	{
		return preCondition;
	}

	void AbstractPlan::setPreCondition(const PreCondition* preCondition)
	{
		this->preCondition = preCondition;
	}

	const UtilityFunction& AbstractPlan::getUtilityFunction() const
	{
		return utilityFunction;
	}

	void AbstractPlan::setUtilityFunction(const UtilityFunction& utilityFunction)
	{
		this->utilityFunction = utilityFunction;
	}

	double AbstractPlan::getUtilityThreshold() const
	{
		return utilityThreshold;
	}

	void AbstractPlan::setUtilityThreshold(double utilityThreshold)
	{
		this->utilityThreshold = utilityThreshold;
	}

} /* namespace Alica */

