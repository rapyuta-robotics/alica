/*
 * CNPosition.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPosition.h"
#include <sstream>
namespace geometry
{

	CNPosition::CNPosition(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPosition::~CNPosition()
	{
	}

	string CNPosition::toString()
	{
		stringstream ss;
		ss << "CNPosition: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}

	double geometry::CNPosition::distanceTo(shared_ptr<CNPoint2D> point)
	{
		return sqrt(pow(this->x - point->x, 2) + pow(this->y - point->y, 2));
	}
}

