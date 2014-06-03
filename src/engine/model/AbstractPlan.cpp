/*
 * AbstractPlan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/AbstractPlan.h"

namespace alica
{

	AbstractPlan::AbstractPlan()
	{
		this->masterPlan = false;
	}

	AbstractPlan::~AbstractPlan()
	{
	}

	bool AbstractPlan::isMasterPlan() const
	{
		return masterPlan;
	}

	void AbstractPlan::setMasterPlan(bool masterPlan)
	{
		this->masterPlan = masterPlan;
	}

	string AbstractPlan::toString() const
	{
		stringstream ss;
		ss << AlicaElement::toString();
		ss << "IsMasterPlan: " << isMasterPlan() << endl;
		return ss.str();
	}

} /* namespace Alica */

