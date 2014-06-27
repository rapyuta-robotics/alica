/*
 * Assignment.cpp
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#include "engine/Assignment.h"

namespace alica
{

	Assignment::Assignment()
	{
		// TODO Auto-generated constructor stub

	}

	Assignment::~Assignment()
	{
		// TODO Auto-generated destructor stub
	}

	Plan* Assignment::getPlan()
	{
		return plan;
	}

	void Assignment::setPlan(Plan* plan)
	{
		this->plan = plan;
	}
	StateCollection* Assignment::getRobotStateMapping()
	{
		return robotStateMapping;
	}


} /* namespace alica */

