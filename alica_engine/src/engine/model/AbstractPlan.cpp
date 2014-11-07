/*
 * AbstractPlan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"

namespace alica
{

	AbstractPlan::AbstractPlan() :
			AlicaElement()
	{
		this->masterPlan = false;
		this->variables = make_shared<list<Variable*> >();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->authorityTimeInterval = (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection","MinimalAuthorityTimeInterval") * 1000000;
		this->utilityFunction = nullptr;
		this->preCondition = nullptr;
		this->runtimeCondition = nullptr;
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

	/**
	 * Tests whether a given variable belongs to this plan.
	 * @param v A Variable*
	 * @return A bool
	 */
	bool AbstractPlan::containsVar(const Variable* v)
	{
		auto vars = this->getVariables();
		return find(vars->begin(), vars->end(), v) != vars->end();
	}

	bool AbstractPlan::containsVar(string name)
	{
		auto vars = this->getVariables();
		for (Variable* v : *vars)
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

	alicaTime AbstractPlan::getAuthorityTimeInterval() const
	{
		return authorityTimeInterval;
	}

	void AbstractPlan::setAuthorityTimeInterval(alicaTime authorithyTimeInterval)
	{
		this->authorityTimeInterval = authorithyTimeInterval;
	}

	const string& AbstractPlan::getFileName() const
	{
		return fileName;
	}

	void AbstractPlan::setFileName(const string& fileName)
	{
		this->fileName = fileName;
	}

	shared_ptr<list<Variable*>> AbstractPlan::getVariables()
	{
		return variables;
	}


	void AbstractPlan::setVariables(shared_ptr<list<Variable*>> variables)
	{
		this->variables = variables;
	}

	RuntimeCondition* AbstractPlan::getRuntimeCondition()
	{
		return runtimeCondition;
	}

	void AbstractPlan::setRuntimeCondition(RuntimeCondition* runtimeCondition)
	{
		this->runtimeCondition = runtimeCondition;
	}

	PreCondition* AbstractPlan::getPreCondition()
	{
		return preCondition;
	}

	void AbstractPlan::setPreCondition(PreCondition* preCondition)
	{
		this->preCondition = preCondition;
	}

	shared_ptr<UtilityFunction> AbstractPlan::getUtilityFunction()
	{
		return utilityFunction;
	}

	void AbstractPlan::setUtilityFunction(shared_ptr<UtilityFunction> utilityFunction)
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

