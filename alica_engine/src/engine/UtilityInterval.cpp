/*
 * UtilityInterval.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#include <engine/UtilityInterval.h>

namespace alica
{
	/**
	 * Holds a minimal and a maximal possible utility value. Used internally for searching.
	 */
	UtilityInterval::UtilityInterval(double min, double max)
	{
		this->min = min;
		this->max = max;
	}

	UtilityInterval::~UtilityInterval()
	{
	}

	/**
	 * The maximally achievable utility.
	 */
	double UtilityInterval::getMax()
	{
		return max;
	}

	void UtilityInterval::setMax(double max)
	{
		this->max = max;
	}

	/**
	 * The minimally achievable utility.
	 */
	double UtilityInterval::getMin()
	{
		return min;
	}

	void UtilityInterval::setMin(double min)
	{
		this->min = min;
	}

} /* namespace alica */
