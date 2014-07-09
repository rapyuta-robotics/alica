/*
 * UtilityInterval.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#include <engine/UtilityInterval.h>

namespace alica
{

	UtilityInterval::UtilityInterval(double min, double max)
	{
		this->min = min;
		this->max = max;
	}

	UtilityInterval::~UtilityInterval()
	{
	}

	double UtilityInterval::getMax()
	{
		return max;
	}

	void UtilityInterval::setMax(double max)
	{
		this->max = max;
	}

	double UtilityInterval::getMin()
	{
		return min;
	}

	void UtilityInterval::setMin(double min)
	{
		this->min = min;
	}

} /* namespace alica */
