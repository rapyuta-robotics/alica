/*
 * PlanType.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PlanType.h"
#include "engine/model/Plan.h"

namespace alica
{

	PlanType::PlanType()
	{

	}

	PlanType::~PlanType()
	{
	}

	string PlanType::toString()
	{
		stringstream ss;
		ss << "#PlanType: " << this->name << " " << this->id << endl;
		ss << "\t Plans: " << this->plans.size() << endl;
		if(this->plans.size() != 0)
		{
			for(Plan* p : this->plans)
			{
				ss << "\t" << p->getId()  << " " << p->getName() << endl;
			}

		}
		ss << "#EndPlanType" << endl;
		return ss.str();
	}

//====================== Getter and Setter =========================

	const string& PlanType::getFileName() const
	{
		if(this->fileName.empty())
		{
			static string result = this->name+".pty";
			return result;
		}
		else
		{
			return this->fileName;
		}
	}

	const list<Parametrisation*>& PlanType::getParametrisation() const
	{
		return parametrisation;
	}

	void PlanType::setParametrisation(const list<Parametrisation*>& parametrisation)
	{
		this->parametrisation = parametrisation;
	}

	const list<Plan*>& PlanType::getPlans() const
	{
		return plans;
	}

	void PlanType::setPlans(const list<Plan*>& plans)
	{
		this->plans = plans;
	}

} /* namespace Alica */


