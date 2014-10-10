/*
 * Point2D.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#include "Point2D.h"

namespace carpenoctem
{
	namespace containers
	{
		Point2D::Point2D(double x, double y)
		{
			_x = x;
			_y = y;
		}

		double Point2D::getX()
		{
			return _x;
		}

		void Point2D::setX(double x)
		{
			_x = x;
		}

		double Point2D::getY()
		{
			return _y;
		}

		void Point2D::setY(double y)
		{
			_y = y;
		}
	} /* namespace containers */
} /* namespace carpenoctem */
