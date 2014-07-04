/*
 * PartialAssignment.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/PartialAssignment.h>

#include "engine/planselector/EpByTaskComparer.h"

namespace alica
{

	PartialAssignment::PartialAssignment()
	{
		this->maxCount = 10100;
		this->maxEpsCount = 20;
		this->curIndex = 0;
		this->daPAs = vector<PartialAssignment*>(maxCount);
		this->epByTaskComparer = EpByTaskComparer();
		this->allowIdling = (*supplementary::SystemConfig::getInstance())["Alica"]->get<bool>("Alica.AllowIdling");


	}

	PartialAssignment::~PartialAssignment()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace alica */
