/*
 * PlanType.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PlanType.h"

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
		return ss.str();
//		ret += "#PlanType: " + this.Name + " " + this.Id + "\n";
//
//		ret += "\tPlans: " +this.Plans.Count+ "\n";
//		if(this.Plans.Count != 0)
//		{
//			foreach (Plan p in this.Plans)
//			{
//				//ret += "\t" + p;
//				ret += "\t" + p.Id + " " + p.Name + "\n";
//			}
//		}
//		ret += "#EndPlanType\n";
//
//		return ret;
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


