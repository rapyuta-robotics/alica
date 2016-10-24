#include <sstream>

#include "container/CNPositionBase.h"

namespace geometry
{

	CNPositionBase::CNPositionBase(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPositionBase::~CNPositionBase() { }

	string CNPositionBase::toString()
	{
		stringstream ss;
		ss << "CNPositionBase: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}

	shared_ptr<CNPoint2D> CNPositionBase::toPoint()
	{
		return make_shared<CNPoint2D>(this->x, this->y);
	}
	
}
