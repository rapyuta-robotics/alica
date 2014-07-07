/*
 * EpByTaskComparer.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/EpByTaskComparer.h>

#include "engine/model/Task.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

	EpByTaskComparer::EpByTaskComparer()
	{
		// TODO Auto-generated constructor stub

	}

	EpByTaskComparer::~EpByTaskComparer()
	{
		// TODO Auto-generated destructor stub
	}

	bool EpByTaskComparer::compareTo(EntryPoint* x, EntryPoint* y)
	{
		return (x->getTask()->getId() < y->getTask()->getId());
	}

} /* namespace alica */
