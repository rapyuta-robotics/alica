/*
 * Parametrisation.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Parametrisation.h"

namespace alica
{

	Parametrisation::Parametrisation()
	{

	}

	Parametrisation::~Parametrisation()
	{
	}


	string Parametrisation::ToString()
	{
		stringstream ss;
		ss << "[Parametrisation: Var=" << this->var.getId();
		ss << " SubVar=" << this->subVar.getName() << " (" << this->subVar.getName() << "), ";
		ss << "SubPlan=" << this->subPlan.getName() << "]" << endl;
		return ss.str();
	}

//================= Getter and Setter ========================

	const AbstractPlan& Parametrisation::getSubPlan() const
	{
		return subPlan;
	}

	void Parametrisation::setSubPlan(const AbstractPlan& subPlan)
	{
		this->subPlan = subPlan;
	}

	const Variable& Parametrisation::getSubVar() const
	{
		return subVar;
	}

	void Parametrisation::setSubVar(const Variable& subVar)
	{
		this->subVar = subVar;
	}

	const Variable& Parametrisation::getVar() const
	{
		return var;
	}

	void Parametrisation::setVar(const Variable& var)
	{
		this->var = var;
	}

} /* namespace Alica */
